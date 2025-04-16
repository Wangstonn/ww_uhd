#include <zmq.hpp>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include "/usr/include/pmt/pmt.h"  // Include PMT header

int main() {
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);

    publisher.bind("tcp://*:5555");  // Bind to the port

    // Give time for GNU Radio to connect
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // Create a PMT object (example: a simple string)
    pmt::pmt_t pmt_msg = pmt::from_double(1.1);

    // Serialize the PMT object (convert PMT to raw byte stream)
    std::string serialized_msg = pmt::serialize_str(pmt_msg);

    // Send the serialized PMT message over ZeroMQ
    zmq::message_t message(serialized_msg.begin(), serialized_msg.end());
    publisher.send(message, zmq::send_flags::none);

    std::cout << "Message sent." << std::endl;

    return 0;
}