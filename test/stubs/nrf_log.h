/* Stand-in for the nRF5 SDK logger: forwards to nrf_log_stub (defined by the host tests). */
#ifndef NRF_LOG_H_STUB
#define NRF_LOG_H_STUB

void nrf_log_stub(const char *fmt, ...);

#define NRF_LOG_INFO(...)    nrf_log_stub(__VA_ARGS__)
#define NRF_LOG_WARNING(...) nrf_log_stub(__VA_ARGS__)
#define NRF_LOG_ERROR(...)   nrf_log_stub(__VA_ARGS__)
#define NRF_LOG_DEBUG(...)   nrf_log_stub(__VA_ARGS__)

#endif
