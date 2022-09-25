use crate::commands::{AccessPointConnectCommand, WifiModeCommand};
use crate::urc::URCMessages;
use atat::{AtatClient, Error as AtError};

/// Central client for network communication
pub struct Adapter<A: AtatClient> {
    /// ATAT client
    pub(crate) client: A,

    /// Currently joined to WIFI network? Gets updated by URC messages.
    joined: bool,

    /// True if an IP was assigned by access point. Get updated by URC message.
    ip_assigned: bool,
}

/// Possible errors when joining an access point
#[derive(Clone, Debug, PartialEq)]
pub enum JoinError {
    /// Error while setting the flash configuration mode
    ConfigurationStoreError(AtError),

    /// Error wile setting WIFI mode to station
    ModeError(AtError),

    /// Error while setting WIFI credentials
    ConnectError(AtError),

    /// Given SSD is longer then the max. size of 32 chars
    InvalidSSDLength,

    /// Given password is longer then the max. size of 63 chars
    InvalidPasswordLength,

    /// Received an unexpected WouldBlock. The most common cause of errors is an incorrect mode of the client.
    /// This must be either timeout or blocking.
    UnexpectedWouldBlock,
}

/// Current WIFI connection state
#[derive(Copy, Clone, Debug)]
pub struct JoinState {
    /// True if connected to an WIFI access point
    pub connected: bool,

    /// True if an IP was assigned
    pub ip_assigned: bool,
}

impl<A: AtatClient> Adapter<A> {
    /// Creates a new network adapter. Client needs to be in timeout or blocking mode
    pub fn new(client: A) -> Self {
        Self {
            client,
            joined: false,
            ip_assigned: false,
        }
    }

    /// Connects to an WIFI access point and returns the connection state
    ///
    /// Note:
    /// If the connection was not successful or is lost, the ESP-AT will try independently fro time
    /// to time (by default every second) to establish connection to the network. The status can be
    /// queried using `get_join_state()`.
    pub fn join(&mut self, ssid: &str, key: &str) -> Result<JoinState, JoinError> {
        self.set_station_mode()?;
        self.connect_access_point(ssid, key)?;
        self.process_urc_messages();

        Ok(JoinState {
            connected: self.joined,
            ip_assigned: self.ip_assigned,
        })
    }

    /// Processes all pending messages in the queue
    pub fn process_urc_messages(&mut self) {
        while self.handle_single_urc() {}
    }

    /// Checks a single pending URC message. Returns false, if no URC message is pending
    fn handle_single_urc(&mut self) -> bool {
        match self.client.check_urc::<URCMessages>() {
            Some(URCMessages::WifiDisconnected) => {
                self.joined = false;
                self.ip_assigned = false;
            }
            Some(URCMessages::ReceivedIP) => self.ip_assigned = true,
            Some(URCMessages::WifiConnected) => self.joined = true,
            Some(URCMessages::Ready) => {}
            Some(URCMessages::Unknown) => {}
            None => return false,
        };

        true
    }

    /// Sends the command for switching to station mode
    fn set_station_mode(&mut self) -> Result<(), JoinError> {
        let command = WifiModeCommand::station_mode();
        if let nb::Result::Err(error) = self.client.send(&command) {
            return match error {
                nb::Error::Other(other) => Err(JoinError::ModeError(other)),
                nb::Error::WouldBlock => Err(JoinError::UnexpectedWouldBlock),
            };
        }

        Ok(())
    }

    /// Sends the command for setting the WIFI credentials
    fn connect_access_point(&mut self, ssid: &str, key: &str) -> Result<(), JoinError> {
        if ssid.len() > 32 {
            return Err(JoinError::InvalidSSDLength);
        }

        if key.len() > 63 {
            return Err(JoinError::InvalidPasswordLength);
        }

        let command = AccessPointConnectCommand::new(ssid.into(), key.into());
        match self.client.send(&command) {
            Ok(_) => Ok(()),
            Err(error) => match error {
                nb::Error::Other(other) => Err(JoinError::ConnectError(other)),
                nb::Error::WouldBlock => Err(JoinError::UnexpectedWouldBlock),
            },
        }
    }
}
