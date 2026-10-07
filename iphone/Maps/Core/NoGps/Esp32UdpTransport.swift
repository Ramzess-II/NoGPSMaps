import Network

/// The UDP connection to the NoGPS ESP32 sensor box for the navigation without GPS in the core: the core talks to the
/// box, this class only sends its lines and passes the received datagrams back on the main queue.
@objc
final class Esp32UdpTransport: NSObject {
  private enum Constants {
    static let retryDelay: TimeInterval = 1
  }

  private var connection: NWConnection?
  private var host = ""
  private var port: UInt16 = 0

  @objc
  func open(host: String, port: UInt16) {
    close()
    LOG(.info, "address = \(host):\(port)")
    self.host = host
    self.port = port
    connect()
  }

  @objc
  func send(_ line: Data) {
    connection?.send(content: line, completion: .contentProcessed { error in
      if let error {
        LOG(.warning, "Send failed: \(error)")
      }
    })
  }

  @objc
  func close() {
    connection?.cancel()
    connection = nil
  }

  private func connect() {
    guard let port = NWEndpoint.Port(rawValue: port) else { return }
    // The box has its own Wi-Fi network without internet: the datagrams must not go over the mobile data.
    let parameters = NWParameters.udp
    parameters.requiredInterfaceType = .wifi
    let connection = NWConnection(host: NWEndpoint.Host(host), port: port, using: parameters)
    self.connection = connection
    connection.stateUpdateHandler = { [weak self, weak connection] state in
      guard let self, let connection, connection === self.connection else { return }
      switch state {
      case .ready:
        self.receive(on: connection)
      case let .failed(error):
        // E.g. the Wi-Fi of the box is not joined yet: try again, the core keeps sending its hello.
        LOG(.warning, "Connection failed: \(error)")
        connection.cancel()
        DispatchQueue.main.asyncAfter(deadline: .now() + Constants.retryDelay) { [weak self] in
          guard let self, self.connection === connection else { return }
          self.connect()
        }
      default:
        break
      }
    }
    connection.start(queue: .main)
  }

  private func receive(on connection: NWConnection) {
    connection.receiveMessage { [weak self, weak connection] data, _, _, error in
      guard let self, let connection, connection === self.connection else { return }
      if let data, !data.isEmpty {
        NoGps.onEsp32Datagram(data)
      }
      if error == nil {
        self.receive(on: connection)
      }
    }
  }
}
