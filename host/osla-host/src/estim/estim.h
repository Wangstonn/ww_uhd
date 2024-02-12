#ifndef ESTIM_H
#define ESTIM_H
//The only time you should include a header within another .h file is if you need to access a type definition in that header
#include <cstdint>
#include <uhd/usrp/multi_usrp.hpp>

//Will eventually need xcorr in here

void compensateDelays(const uhd::usrp::multi_usrp::sptr tx_usrp, const int D_hat);
template <typename T>
std::vector<T> upsample(const std::vector<T>& input, int N);


#endif  // ESTIM_H
