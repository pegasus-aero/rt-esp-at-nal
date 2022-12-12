//! # TCP client stack
//!
//! This crate fully implements [TcpClientStack] of [embedded_nal].
//!
//! Block/chunk size is defined a const generics, s. [Adapter] for more details.
//!
//! ## Example
//!
//! ````
//! # use core::str::FromStr;
//! # use embedded_nal::{SocketAddr, TcpClientStack};
//! # use esp_at_nal::example::ExampleTimer;
//! # use esp_at_nal::wifi::{Adapter, WifiAdapter};
//! # use crate::esp_at_nal::example::ExampleAtClient as AtClient;
//! #
//! let client = AtClient::default();
//! let mut adapter: Adapter<_, _, 1_000_000, 1024, 1024> = Adapter::new(client, ExampleTimer::default());
//!
//! // Creating a TCP connection
//! let mut  socket = adapter.socket().unwrap();
//! adapter.connect(&mut socket, SocketAddr::from_str("10.0.0.1:21").unwrap()).unwrap();
//!
//! // Sending some data
//! adapter.send(&mut socket, b"hallo!").unwrap();
//!
//! // Receiving some data
//! let mut  rx_buffer = [0x0; 64];
//! let length = adapter.receive(&mut socket, &mut rx_buffer).unwrap();
//! assert_eq!(16, length);
//! assert_eq!(b"nice to see you!", &rx_buffer[..16]);
//!
//! // Closing socket
//! adapter.close(socket).unwrap();
//! ````
use crate::commands::{
    CloseSocketCommand, ConnectCommand, ReceiveDataCommand, SetMultipleConnectionsCommand,
    SetSocketReceivingModeCommand, TransmissionCommand, TransmissionPrepareCommand,
};
use crate::wifi::{Adapter, Session};
use atat::AtatClient;
use atat::Error as AtError;
use embedded_nal::{SocketAddr, TcpClientStack};
use fugit_timer::Timer;
use heapless::Vec;

/// Unique socket for a network connection
#[derive(Debug)]
pub struct Socket {
    /// Unique link id of AT
    #[allow(unused)]
    pub(crate) link_id: usize,
}

impl Socket {
    pub(crate) fn new(link_id: usize) -> Self {
        Self { link_id }
    }
}

/// Internal state of a single socket
#[derive(Copy, Clone, Default)]
pub(crate) struct SocketState {
    /// Connection state
    pub(crate) state: ConnectionState,

    /// Data length in bytes available to receive which is buffered by ESP-AT
    pub(crate) data_available: usize,
}

/// Internal connection state
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum ConnectionState {
    /// Socket is closed an may be (re)used
    Closed,
    /// Socket was returned by socket() but is not connected yet
    Open,
    /// Connection is fully open
    Connected,
    /// Socket was closed by URC message, but Socket object still exists and needs to be fully closed by calling 'close()'
    Closing,
}

impl Default for ConnectionState {
    fn default() -> Self {
        Self::Closed
    }
}

/// Network related errors
#[derive(Clone, Debug, PartialEq)]
pub enum Error {
    /// Error while sending CIPMUX command for enabling multiple connections
    EnablingMultiConnectionsFailed(AtError),

    /// Error while sending CIPRECVMODE command for enabling passive socket receiving mode
    EnablingPassiveSocketModeFailed(AtError),

    /// TCP connect command failed
    ConnectError(AtError),

    /// Preparing the transmission failed (CIPSEND command)
    TransmissionStartFailed(AtError),

    /// Transmission of data failed
    SendFailed(AtError),

    /// Transmission of data failed
    ReceiveFailed(AtError),

    /// Socket close command failed
    CloseError(AtError),

    /// AT-ESP confirmed receiving an unexpected byte count
    PartialSend,

    /// TCP connect or close command was responded by by OK. But connect or close was not confirmed by URC message.
    UnconfirmedSocketState,

    /// No socket available, since the maximum number is in use.
    NoSocketAvailable,

    /// Given socket is already connected to another remote. Socket needs to be closed first.
    AlreadyConnected,

    /// Unable to send data if socket is not connected
    SocketUnconnected,

    /// Socket was remotely closed and needs to either reconnected to fully closed by calling `close()` for [Adapter]
    ClosingSocket,

    /// Received more data then requested from AT-ESP and data does not fit in (remaining) buffer.
    /// This indicates either a bug in this crate or in AT-ESP firmware.
    ReceiveOverflow,

    /// Received an unexpected WouldBlock. The most common cause of errors is an incorrect mode of the client.
    /// This must be either timeout or blocking.
    UnexpectedWouldBlock,

    /// Upstream timer error
    TimerError,
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        match self {
            Error::EnablingMultiConnectionsFailed(e) => {
                defmt::write!(f, "Error::EnablingMultiConnectionsFailed({})", e)
            }
            Error::EnablingPassiveSocketModeFailed(e) => {
                defmt::write!(f, "Error::EnablingPassiveSocketModeFailed({})", e)
            }
            Error::ConnectError(e) => defmt::write!(f, "Error::ConnectError({})", e),
            Error::TransmissionStartFailed(e) => defmt::write!(f, "Error::TransmissionStartFailed({})", e),
            Error::SendFailed(e) => defmt::write!(f, "Error::SendFailed({})", e),
            Error::ReceiveFailed(e) => defmt::write!(f, "Error::ReceiveFailed({})", e),
            Error::CloseError(e) => defmt::write!(f, "Error::CloseError({})", e),
            Error::PartialSend => defmt::write!(f, "Error::PartialSend"),
            Error::UnconfirmedSocketState => defmt::write!(f, "Error::UnconfirmedSocketState"),
            Error::NoSocketAvailable => defmt::write!(f, "Error::NoSocketAvailable"),
            Error::AlreadyConnected => defmt::write!(f, "Error::AlreadyConnected"),
            Error::SocketUnconnected => defmt::write!(f, "Error::SocketUnconnected"),
            Error::ClosingSocket => defmt::write!(f, "Error::ClosingSocket"),
            Error::ReceiveOverflow => defmt::write!(f, "Error::ReceiveOverflow"),
            Error::UnexpectedWouldBlock => defmt::write!(f, "Error::UnexpectedWouldBlock"),
            Error::TimerError => defmt::write!(f, "Error::TimerError"),
        }
    }
}

impl<A: AtatClient, T: Timer<TIMER_HZ>, const TIMER_HZ: u32, const TX_SIZE: usize, const RX_SIZE: usize> TcpClientStack
    for Adapter<A, T, TIMER_HZ, TX_SIZE, RX_SIZE>
{
    type TcpSocket = Socket;
    type Error = Error;

    /// Opens and returns a new socket
    /// Currently only five parallel sockets are supported. If not socket is available [Error::NoSocketAvailable] is returned.
    ///
    /// On first call ESP-AT is configured to support multiple connections.
    fn socket(&mut self) -> Result<Self::TcpSocket, Self::Error> {
        self.enable_multiple_connections()?;
        self.open_socket()
    }

    /// Opens a new TCP connection. Both IPv4 and IPv6 are supported.
    /// Returns [Error::AlreadyConnected] if socket is already connected.
    ///
    /// On first call ESP-AT is configured for passive socket receiving mode. So receiving data
    /// is buffered on ESP-AT to a maximum size of around 8192 bytes.
    fn connect(&mut self, socket: &mut Socket, remote: SocketAddr) -> nb::Result<(), Self::Error> {
        self.process_urc_messages();

        if self.session.is_socket_connected(socket) {
            return nb::Result::Err(nb::Error::Other(Error::AlreadyConnected));
        }

        self.enable_passive_receiving_mode()?;
        self.session.already_connected = false;

        let command = match remote {
            SocketAddr::V4(address) => ConnectCommand::tcp_v4(socket.link_id, address),
            SocketAddr::V6(address) => ConnectCommand::tcp_v6(socket.link_id, address),
        };
        let result = self.send_command(command);
        self.process_urc_messages();

        // ESP-AT returned that given socket is already connected. This indicates that a URC Connect message was missed.
        if self.session.already_connected {
            self.session.sockets[socket.link_id].state = ConnectionState::Connected;
            return nb::Result::Ok(());
        }
        result?;

        if !self.session.is_socket_connected(socket) {
            return nb::Result::Err(nb::Error::Other(Error::UnconfirmedSocketState));
        }

        self.session.reset_available_data(socket);
        nb::Result::Ok(())
    }

    /// Returns true if the socket is currently connected. Connection aborts by the remote side are also taken into account.
    /// The current implementation never returns a Error.
    fn is_connected(&mut self, socket: &Self::TcpSocket) -> Result<bool, Self::Error> {
        self.process_urc_messages();
        Ok(self.session.is_socket_connected(socket))
    }

    /// Sends the given buffer and returns the length (in bytes) sent.
    /// The data is divided into smaller blocks. The block size is determined by the generic constant TX_SIZE.
    fn send(&mut self, socket: &mut Socket, buffer: &[u8]) -> nb::Result<usize, Error> {
        self.process_urc_messages();
        self.assert_socket_connected(socket)?;

        for chunk in buffer.chunks(TX_SIZE) {
            self.send_command(TransmissionPrepareCommand::new(socket.link_id, chunk.len()))?;
            self.send_chunk(chunk)?;
        }

        nb::Result::Ok(buffer.len())
    }

    /// Receives data (if available) and writes it to the given buffer.
    ///
    /// The data is read internally in blocks. The block size is defined by the generic constant RX_SIZE.
    /// In any case, data is read until the buffer is completely filled or no further data is available.
    fn receive(&mut self, socket: &mut Self::TcpSocket, buffer: &mut [u8]) -> nb::Result<usize, Self::Error> {
        self.process_urc_messages();

        if !self.session.is_data_available(socket) {
            return nb::Result::Err(nb::Error::WouldBlock);
        }

        let mut buffer: Buffer<RX_SIZE> = Buffer::new(buffer);

        while self.session.is_data_available(socket) && !buffer.is_full() {
            let command = ReceiveDataCommand::<RX_SIZE>::new(socket.link_id, buffer.get_next_length());
            self.send_command(command)?;
            self.process_urc_messages();

            if self.session.data.is_none() {
                return nb::Result::Err(nb::Error::Other(Error::ReceiveFailed(AtError::InvalidResponse)));
            }

            let data = self.session.data.take().unwrap();
            self.session.reduce_available_data(socket, data.len());
            buffer.append(data)?;
        }

        nb::Result::Ok(buffer.len())
    }

    /// Closes a socket
    ///
    /// If the socket has already been closed by the remote side or is not connected, no command
    /// is sent to the ESP-AT but only the internal status is set.
    /// In case of an error (which is returned) the socket is internally set to closed so that it is not lost and can be reused.
    fn close(&mut self, socket: Self::TcpSocket) -> Result<(), Self::Error> {
        self.process_urc_messages();

        // Socket already closed during restart
        if self.session.is_socket_closed(&socket) {
            return Ok(());
        }

        // Socket is not connected yet or was already closed remotely
        if self.session.is_socket_closing(&socket) || self.session.is_socket_open(&socket) {
            self.session.sockets[socket.link_id].state = ConnectionState::Closed;
            return Ok(());
        }

        let mut result = self.send_command(CloseSocketCommand::new(socket.link_id));
        self.process_urc_messages();

        if !self.session.is_socket_closing(&socket) && result.is_ok() {
            result = Err(Error::UnconfirmedSocketState);
        }

        // Setting to Closed even on error. Otherwise socket can not be reused in future, as its consumed.
        self.session.sockets[socket.link_id].state = ConnectionState::Closed;

        result?;
        Ok(())
    }
}

impl<A: AtatClient, T: Timer<TIMER_HZ>, const TIMER_HZ: u32, const TX_SIZE: usize, const RX_SIZE: usize>
    Adapter<A, T, TIMER_HZ, TX_SIZE, RX_SIZE>
{
    /// Sends a chunk of max. 256 bytes
    fn send_chunk(&mut self, data: &[u8]) -> Result<(), Error> {
        self.session.send_confirmed = None;
        self.session.recv_byte_count = None;

        self.send_command::<TransmissionCommand<'_>, TX_SIZE>(TransmissionCommand::new(data))?;
        self.timer.start(self.send_timeout).map_err(|_| Error::TimerError)?;

        while self.session.send_confirmed.is_none() {
            self.process_urc_messages();

            if let Some(send_success) = self.session.send_confirmed {
                // Transmission failed
                if !send_success {
                    // Reset prompt status. Otherwise client does not match any command responses.
                    self.client.reset();
                    return Err(Error::SendFailed(AtError::Error));
                }

                // Byte count does not match
                if self.session.is_received_byte_count_incorrect(data.len()) {
                    return Err(Error::PartialSend);
                }

                return Ok(());
            }

            match self.timer.wait() {
                Ok(_) => {
                    // Reset prompt status. Otherwise client does not match any command responses.
                    self.client.reset();
                    return Err(Error::SendFailed(AtError::Timeout));
                }
                Err(error) => match error {
                    nb::Error::Other(_) => return Err(Error::TimerError),
                    nb::Error::WouldBlock => {}
                },
            }
        }

        Ok(())
    }

    /// Enables multiple connections.
    /// Stores internal state, so command is just sent once for saving bandwidth
    fn enable_multiple_connections(&mut self) -> Result<(), Error> {
        if self.session.multi_connections_enabled {
            return Ok(());
        }

        self.send_command(SetMultipleConnectionsCommand::multiple())?;
        self.session.multi_connections_enabled = true;
        Ok(())
    }

    /// Enables the passive socket receiving mode
    /// Stores internal state, so command is just sent once for saving bandwidth
    fn enable_passive_receiving_mode(&mut self) -> Result<(), Error> {
        if self.session.passive_mode_enabled {
            return Ok(());
        }

        self.send_command(SetSocketReceivingModeCommand::passive_mode())?;
        self.session.passive_mode_enabled = true;
        Ok(())
    }

    /// Assigns a free link_id. Returns an error in case no more free sockets are available
    fn open_socket(&mut self) -> Result<Socket, Error> {
        if let Some(link_id) = self.session.get_next_open() {
            self.session.sockets[link_id].state = ConnectionState::Open;
            return Ok(Socket::new(link_id));
        }

        Err(Error::NoSocketAvailable)
    }

    /// Asserts that the given socket is connected and returns otherwise the appropriate error
    fn assert_socket_connected(&self, socket: &Socket) -> nb::Result<(), Error> {
        if self.session.is_socket_closing(socket) {
            return nb::Result::Err(nb::Error::Other(Error::ClosingSocket));
        }

        if !self.session.is_socket_connected(socket) {
            return nb::Result::Err(nb::Error::Other(Error::SocketUnconnected));
        }

        nb::Result::Ok(())
    }
}

impl<const RX_SIZE: usize> Session<RX_SIZE> {
    /// Fetches the next open socket ID and returns None in case no socket is available
    fn get_next_open(&self) -> Option<usize> {
        self.sockets.iter().position(|state| state.state == ConnectionState::Closed)
    }

    /// Returns true if data is available for the given socket
    fn is_data_available(&self, socket: &Socket) -> bool {
        self.sockets[socket.link_id].data_available > 0
    }

    /// Reduces the available data length mark by the given length of the given socket ID
    fn reduce_available_data(&mut self, socket: &Socket, length: usize) {
        if self.sockets[socket.link_id].data_available < length {
            self.sockets[socket.link_id].data_available = 0;
            return;
        }

        self.sockets[socket.link_id].data_available -= length;
    }

    /// Returns true if the reported received byte length does NOT match the actual data length
    /// Returns false if received byte count was not reported by ESP-AT (older firmware version)
    fn is_received_byte_count_incorrect(&self, actual_data_length: usize) -> bool {
        self.recv_byte_count.is_some() && *self.recv_byte_count.as_ref().unwrap() != actual_data_length
    }

    /// Sets the available data of the given socket to zero
    fn reset_available_data(&mut self, socket: &Socket) {
        self.sockets[socket.link_id].data_available = 0;
    }

    /// Returns true if the given socket is in OPEN state
    fn is_socket_open(&self, socket: &Socket) -> bool {
        self.sockets[socket.link_id].state == ConnectionState::Open
    }

    /// Returns true if the given socket is in CLOSED state
    fn is_socket_closed(&self, socket: &Socket) -> bool {
        self.sockets[socket.link_id].state == ConnectionState::Closed
    }

    /// Returns true if the given socket is in CLOSING state
    fn is_socket_closing(&self, socket: &Socket) -> bool {
        self.sockets[socket.link_id].state == ConnectionState::Closing
    }

    /// Returns true if the given socket is in CONNECTED state
    fn is_socket_connected(&self, socket: &Socket) -> bool {
        self.sockets[socket.link_id].state == ConnectionState::Connected
    }
}

/// Helper for filling receive buffer
pub(crate) struct Buffer<'a, const CHUNK_SIZE: usize> {
    buffer: &'a mut [u8],

    /// Next buffer index to start inserting data
    position: usize,
}

impl<'a, const CHUNK_SIZE: usize> Buffer<'a, CHUNK_SIZE> {
    pub fn new(buffer: &'a mut [u8]) -> Self {
        Self { buffer, position: 0 }
    }

    /// Returns the length of next chunk based on max. chunk_size and available buffer space
    pub fn get_next_length(&self) -> usize {
        let buffer_space = self.buffer_space();

        if buffer_space > CHUNK_SIZE {
            return CHUNK_SIZE;
        }

        buffer_space
    }

    /// Appends the response to the buffer
    pub fn append(&mut self, data: Vec<u8, CHUNK_SIZE>) -> Result<(), Error> {
        if data.len() > self.buffer_space() {
            return Err(Error::ReceiveOverflow);
        }

        let end = self.position + data.len();

        self.buffer[self.position..end].copy_from_slice(data.as_slice());
        self.position = end;
        Ok(())
    }

    /// Returns true if the buffer is completely filled
    pub fn is_full(&self) -> bool {
        if self.buffer.is_empty() {
            return true;
        }

        self.position >= self.buffer.len()
    }

    /// Returns the remaining free buffer space
    fn buffer_space(&self) -> usize {
        self.buffer.len() - self.position
    }

    /// Returns the current fill length
    pub(crate) fn len(&self) -> usize {
        self.position
    }
}
