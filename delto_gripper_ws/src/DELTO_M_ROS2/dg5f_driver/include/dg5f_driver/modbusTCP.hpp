#include <boost/asio.hpp>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// Linux 전용 헤더
#include <netinet/in.h>
#include <sys/socket.h>

class ModbusClient {
 public:
  // 생성자: 호스트와 포트를 인자로 받아 연결 설정 (기본 타임아웃 200ms)
  explicit ModbusClient(const std::string& host, int port)
      : socket_(io_service_),
        host_(host),
        port_(port),
        transaction_id_(0) {
    setTimeout(200);  // Change from 5000ms to 200ms
  }

  virtual ~ModbusClient() { disconnect(); }

  // 읽기/쓰기 타임아웃 설정 (밀리초 단위)
  void setTimeout(int timeout_ms) {
    try {
      if (socket_.is_open()) {
        struct timeval tv;
        tv.tv_sec = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        setsockopt(socket_.native_handle(), SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
        setsockopt(socket_.native_handle(), SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<const char*>(&tv), sizeof(tv));
      }
    } catch (const std::exception& e) {
      std::cerr << "Error setting timeout: " << e.what() << std::endl;
    }
  }

  // Modbus 서버에 연결 (기본 타임아웃: 5초)
  bool connect(int timeout_seconds = 5) {
    try {
      // Close existing connection if any
      disconnect();
      
      // Create resolver
      boost::asio::ip::tcp::resolver resolver(io_service_);
      boost::system::error_code ec;
      boost::asio::ip::tcp::resolver::query query(host_, std::to_string(port_));
      auto endpoints = resolver.resolve(query, ec);
      
      if (ec) {
        std::cerr << "Resolver error: " << ec.message() << std::endl;
        return false;
      }
      
      // Set up socket
      socket_.open(boost::asio::ip::tcp::v4(), ec);
      if (ec) {
        std::cerr << "Socket open error: " << ec.message() << std::endl;
        return false;
      }
      
      // Set socket options for timeout
      socket_.set_option(boost::asio::socket_base::linger(true, timeout_seconds), ec);
      
      // Use synchronous connect directly
      boost::system::error_code connect_ec;
      boost::asio::connect(socket_, endpoints, connect_ec);
      
      if (connect_ec) {
        std::cerr << "Connection error: " << connect_ec.message() << std::endl;
        socket_.close(ec);
        return false;
      }
      
      // Set timeout for reads/writes
      setTimeout(timeout_seconds * 1000);
      
      std::cout << "Connected to " << host_ << ":" << port_ << std::endl;
      return true;
    } 
    catch (const std::exception& e) {
      std::cerr << "Connection failed: " << e.what() << std::endl;
      return false;
    }
  }

  // 최대 max_attempts 만큼 재연결 시도
  bool reconnect(int max_attempts = 3) {
    for (int i = 0; i < max_attempts; ++i) {
      std::cout << "Reconnection attempt " << (i + 1) << "/" << max_attempts
                << std::endl;
      disconnect();
      if (connect()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return false;
  }

  // 연결 종료
  void disconnect() {
    if (socket_.is_open()) {
      boost::system::error_code ec;
      socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
      socket_.close(ec);
    }
  }

  // 지정 주소부터 quantity 만큼의 홀딩 레지스터 읽기
  std::vector<int16_t> readHoldingRegisters(uint16_t start_address,
                                              uint16_t quantity) {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      uint16_t current_transaction = transaction_id_++;

      std::vector<uint8_t> request = {
          static_cast<uint8_t>(current_transaction >> 8),
          static_cast<uint8_t>(current_transaction & 0xFF), 0x00, 0x00,
          0x00, 0x06, 0x01, 0x03,
          static_cast<uint8_t>(start_address >> 8),
          static_cast<uint8_t>(start_address & 0xFF),
          static_cast<uint8_t>(quantity >> 8),
          static_cast<uint8_t>(quantity & 0xFF)};

      boost::asio::write(socket_, boost::asio::buffer(request));

      std::vector<uint8_t> response(9 + quantity * 2);
      boost::asio::read(socket_, boost::asio::buffer(response));

      // Transaction ID 검증
      uint16_t received_transaction =
          (response[0] << 8) | response[1];
      if (current_transaction != received_transaction) {
        throw std::runtime_error(
            "Transaction ID mismatch in readHoldingRegisters");
      }

      // 함수 코드 검증
      if (response[7] != 0x03) {
        if (response[7] == 0x83) {
          throw std::runtime_error("Modbus exception received");
        }
        throw std::runtime_error("Invalid function code");
      }

      std::vector<int16_t> registers;
      registers.reserve(quantity);
      for (size_t i = 0; i < quantity; ++i) {
        int16_t value = static_cast<int16_t>(
            (response[9 + i * 2] << 8) | response[10 + i * 2]);
        registers.push_back(value);
      }
      return registers;
    } catch (const std::exception& e) {
      std::cerr << "Error in readHoldingRegisters: " << e.what()
                << std::endl;
      throw;
    }
  }

  // 지정 주소부터 quantity 만큼의 입력 레지스터 읽기
  std::vector<int16_t> readInputRegisters(uint16_t start_address,
                                          uint16_t quantity,
                                          unsigned int timeout_ms = 200) {
    try {
      // Set the timeout for this specific operation
      setTimeout(timeout_ms);

      std::lock_guard<std::mutex> lock(mutex_);
      uint16_t current_transaction = transaction_id_++;

      std::vector<uint8_t> request(12);
      request[0] = static_cast<uint8_t>(current_transaction >> 8);
      request[1] = static_cast<uint8_t>(current_transaction & 0xFF);
      request[2] = 0x00;
      request[3] = 0x00;
      request[4] = 0x00;
      request[5] = 0x06;
      request[6] = 0x01;
      request[7] = 0x04;
      request[8] = static_cast<uint8_t>(start_address >> 8);
      request[9] = static_cast<uint8_t>(start_address & 0xFF);
      request[10] = static_cast<uint8_t>(quantity >> 8);
      request[11] = static_cast<uint8_t>(quantity & 0xFF);

      boost::asio::write(socket_, boost::asio::buffer(request));

      std::vector<uint8_t> response(9 + quantity * 2, 0);
      boost::system::error_code ec;
      size_t total_bytes_read = 0;
      auto start_time = std::chrono::steady_clock::now();

      while (total_bytes_read < response.size()) {
        // Check if we've exceeded our timeout
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();
        if (elapsed > timeout_ms) {
          throw std::runtime_error("Read timeout after " + std::to_string(elapsed) + "ms");
        }
        
        // Read what's available, with a small buffer slice for the remaining data
        size_t bytes_read = socket_.read_some(
          boost::asio::buffer(response.data() + total_bytes_read, 
                             response.size() - total_bytes_read), ec);
        
        if (ec) {
          if (ec == boost::asio::error::would_block || ec == boost::asio::error::try_again) {
            // No data available, sleep briefly and retry
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }
          throw std::runtime_error("Socket read error: " + ec.message());
        }
        
        if (bytes_read == 0) {
          // Connection closed
          throw std::runtime_error("Connection closed during read");
        }
        
        total_bytes_read += bytes_read;
      }

      // Transaction ID 검증 및 재시도 (최대 3회)
      uint16_t received_transaction = (response[0] << 8) + response[1];
      if (current_transaction != received_transaction) {
        std::cerr << "Transaction ID mismatch. Expected: "
                  << current_transaction << ", Got: " << received_transaction
                  << std::endl;
        for (int retry = 0; retry < 3; ++retry) {
          std::cout << "Retrying... Attempt " << (retry + 1) << std::endl;
          boost::asio::write(socket_, boost::asio::buffer(request));
          boost::asio::read(socket_, boost::asio::buffer(response));
          received_transaction = (response[0] << 8) + response[1];
          if (received_transaction == current_transaction) {
            std::cout << "Success after retry" << std::endl;
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (received_transaction != current_transaction) {
          throw std::runtime_error("Transaction ID mismatch after retries");
        }
      }

      // 프로토콜 ID 검증
      if (response[2] != 0x00 || response[3] != 0x00) {
        throw std::runtime_error("Invalid protocol ID");
      }

      // 함수 코드 검증
      if (response[7] != 0x04) {
        if (response[7] == 0x84) {
          throw std::runtime_error("Modbus exception received");
        }
        throw std::runtime_error("Invalid function code");
      }

      // 바이트 수 검증
      uint8_t byte_count = response[8];
      if (byte_count != quantity * 2) {
        throw std::runtime_error("Incorrect byte count");
      }

      std::vector<int16_t> registers;
      registers.reserve(quantity);
      for (size_t i = 0; i < quantity; ++i) {
        int16_t value = static_cast<int16_t>(
            (response[9 + i * 2] << 8) | response[10 + i * 2]);
        registers.push_back(value);
      }
      return registers;
    } catch (const std::exception& e) {
      std::cerr << "Error in readInputRegisters: " << e.what() << std::endl;
      throw;
    }
  }

  // 단일 코일 쓰기
  void writeSingleCoil(uint16_t address, bool value) {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      uint16_t current_transaction = transaction_id_++;

      std::vector<uint8_t> request = {
          static_cast<uint8_t>(current_transaction >> 8),
          static_cast<uint8_t>(current_transaction & 0xFF), 0x00, 0x00,
          0x00, 0x06, 0x01, 0x05,
          static_cast<uint8_t>(address >> 8),
          static_cast<uint8_t>(address & 0xFF),
          static_cast<uint8_t>(value ? 0xFF : 0x00), 0x00};

      boost::asio::write(socket_, boost::asio::buffer(request));

      std::vector<uint8_t> response(12);
      boost::asio::read(socket_, boost::asio::buffer(response));

      uint16_t received_transaction =
          (response[0] << 8) | response[1];
      if (current_transaction != received_transaction) {
        throw std::runtime_error("Transaction ID mismatch");
      }

      if (response[7] != 0x05) {
        if (response[7] == 0x85) {
          throw std::runtime_error("Modbus exception received");
        }
        throw std::runtime_error("Invalid function code");
      }
    } catch (const std::exception& e) {
      std::cerr << "Error in writeSingleCoil: " << e.what() << std::endl;
      throw;
    }
  }

  // 다중 코일 쓰기
  void writeMultiCoils(uint16_t address,
                       const std::vector<bool>& values) {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      uint16_t current_transaction = transaction_id_++;

      size_t num_bytes = (values.size() + 7) / 8;
      std::vector<uint8_t> value_bytes(num_bytes, 0x00);
      for (size_t i = 0; i < values.size(); ++i) {
        if (values[i]) {
          value_bytes[i / 8] |= 1 << (i % 8);
        }
      }

      std::vector<uint8_t> request = {
          static_cast<uint8_t>(current_transaction >> 8),
          static_cast<uint8_t>(current_transaction & 0xFF), 0x00, 0x00,
          static_cast<uint8_t>((7 + num_bytes) >> 8),
          static_cast<uint8_t>((7 + num_bytes) & 0xFF),
          0x01, 0x0F,
          static_cast<uint8_t>(address >> 8),
          static_cast<uint8_t>(address & 0xFF),
          static_cast<uint8_t>(values.size() >> 8),
          static_cast<uint8_t>(values.size() & 0xFF),
          static_cast<uint8_t>(num_bytes)};
      request.insert(request.end(), value_bytes.begin(), value_bytes.end());

      boost::asio::write(socket_, boost::asio::buffer(request));

      std::vector<uint8_t> response(12);
      boost::asio::read(socket_, boost::asio::buffer(response));

      uint16_t received_transaction =
          (response[0] << 8) | response[1];
      if (current_transaction != received_transaction) {
        throw std::runtime_error("Transaction ID mismatch");
      }

      if (response[7] != 0x0F) {
        if (response[7] == 0x8F) {
          throw std::runtime_error("Modbus exception received");
        }
        throw std::runtime_error("Invalid function code");
      }
    } catch (const std::exception& e) {
      std::cerr << "Error in writeMultiCoils: " << e.what() << std::endl;
      throw;
    }
  }

  // 단일 레지스터 쓰기
  void writeSingleRegister(uint16_t address, uint16_t value) {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      uint16_t current_transaction = transaction_id_++;

      std::vector<uint8_t> request = {
          static_cast<uint8_t>(current_transaction >> 8),
          static_cast<uint8_t>(current_transaction & 0xFF), 0x00, 0x00,
          0x00, 0x06, 0x01, 0x06,
          static_cast<uint8_t>(address >> 8),
          static_cast<uint8_t>(address & 0xFF),
          static_cast<uint8_t>(value >> 8),
          static_cast<uint8_t>(value & 0xFF)};

      boost::asio::write(socket_, boost::asio::buffer(request));

      std::vector<uint8_t> response(12);
      boost::asio::read(socket_, boost::asio::buffer(response));

      uint16_t received_transaction =
          (response[0] << 8) | response[1];
      if (current_transaction != received_transaction) {
        throw std::runtime_error("Transaction ID mismatch");
      }

      if (response[7] != 0x06) {
        if (response[7] == 0x86) {
          throw std::runtime_error("Modbus exception received");
        }
        throw std::runtime_error("Invalid function code");
      }
    } catch (const std::exception& e) {
      std::cerr << "Error in writeSingleRegister: " << e.what() << std::endl;
      throw;
    }
  }

  // 다중 레지스터 쓰기
  void writeMultiRegisters(uint16_t address,
                           const std::vector<uint16_t>& values) {
    try {
      std::lock_guard<std::mutex> lock(mutex_);
      uint16_t current_transaction = transaction_id_++;

      size_t values_bytes = values.size() * 2;
      std::vector<uint8_t> request = {
          static_cast<uint8_t>(current_transaction >> 8),
          static_cast<uint8_t>(current_transaction & 0xFF), 0x00, 0x00,
          static_cast<uint8_t>((7 + values_bytes) >> 8),
          static_cast<uint8_t>((7 + values_bytes) & 0xFF),
          0x01, 0x10,
          static_cast<uint8_t>(address >> 8),
          static_cast<uint8_t>(address & 0xFF),
          static_cast<uint8_t>(values.size() >> 8),
          static_cast<uint8_t>(values.size() & 0xFF),
          static_cast<uint8_t>(values_bytes)};

      for (const auto& value : values) {
        request.push_back(static_cast<uint8_t>(value >> 8));
        request.push_back(static_cast<uint8_t>(value & 0xFF));
      }

      boost::asio::write(socket_, boost::asio::buffer(request));

      std::vector<uint8_t> response(12);
      boost::asio::read(socket_, boost::asio::buffer(response));

      uint16_t received_transaction =
          (response[0] << 8) | response[1];
      if (current_transaction != received_transaction) {
        throw std::runtime_error("Transaction ID mismatch");
      }

      if (response[7] != 0x10) {
        if (response[7] == 0x90) {
          throw std::runtime_error("Modbus exception received");
        }
        throw std::runtime_error("Invalid function code");
      }
    } catch (const std::exception& e) {
      std::cerr << "Error in writeMultiRegisters: " << e.what() << std::endl;
      throw;
    }
  }

  // 연결 상태 확인
  bool isConnected() const {
    return socket_.is_open();
  }

  // 소켓 버퍼 내 남은 데이터를 제거
  void clearBuffers() {
    if (socket_.is_open()) {
      boost::system::error_code ec;
      socket_.cancel(ec);
      std::vector<uint8_t> buffer(1024);
      int max_iterations = 10; // Prevent infinite loop
      
      while (socket_.available() > 0 && max_iterations-- > 0) {
        std::size_t bytes_read = socket_.read_some(boost::asio::buffer(buffer), ec);
        if (ec || bytes_read == 0) {
          break; // Exit if error or no bytes read
        }
      }
    }
  }

 private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  std::string host_;
  uint16_t port_;
  std::atomic<uint16_t> transaction_id_;
  std::mutex mutex_;

  // Modbus 예외 처리를 위한 헬퍼 함수
  void handleModbusException(uint8_t exception_code) {
    std::string error;
    switch (exception_code) {
      case 0x01:
        error = "Illegal function";
        break;
      case 0x02:
        error = "Illegal data address";
        break;
      case 0x03:
        error = "Illegal data value";
        break;
      case 0x04:
        error = "Slave device failure";
        break;
      case 0x05:
        error = "Acknowledge";
        break;
      case 0x06:
        error = "Slave device busy";
        break;
      case 0x08:
        error = "Memory parity error";
        break;
      case 0x0A:
        error = "Gateway path unavailable";
        break;
      case 0x0B:
        error = "Gateway target device failed to respond";
        break;
      default:
        error = "Unknown error";
        break;
    }
    throw std::runtime_error("Modbus exception: " + error);
  }
};

