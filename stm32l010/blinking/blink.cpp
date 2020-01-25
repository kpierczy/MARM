#include <config/conf.h>
#include <foundation/sys/dbglog.h>
#include <periph/gpio/gpio.hpp>


namespace {
    constexpr auto led_0 = periph::gpio::num::PD13;
}


auto main() -> int
{
    // Configure PD13 pin LED as an output
    periph::gpio::setup( led_0,
        periph::gpio::mode::out{
            periph::gpio::outtype::pushpull,
            periph::gpio::speed::low
        }
    );
    dbprintf("<<<< Hello STM32F411E-DISCO board >>>>");
	return 0;
}
