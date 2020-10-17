// src/main.rs

// std and main are not available for bare metal software
#![no_std]
#![no_main]

use cortex_m_rt::entry; // The runtime
//use cortex_m_semihosting::hprintln;

use nb::block;

use ssd1306::{prelude::*, Builder};


//use embedded_hal::digital::v2::OutputPin; // the `set_high/low`function
use stm32f1xx_hal::{
    delay::Delay, pac, adc, prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
    serial::{Config, Serial},
}; // STM32F1 specific functions

use microfft;

use micromath::F32Ext;



#[allow(unused_imports)]
use panic_halt; // When a panic occurs, stop the microcontroller



// This marks the entrypoint of our application. The cortex_m_rt creates some
// startup code before this, but we don't need to worry about this
#[entry]
fn main() -> ! {

    // Get handles to the hardware objects. These functions can only be called
    // once, so that the borrowchecker can ensure you don't reconfigure
    // something by accident.
    let dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // GPIO pins on the STM32F1 must be driven by the APB2 peripheral clock.
    // This must be enabled first. The HAL provides some abstractions for
    // us: First get a handle to the RCC peripheral:
    let mut rcc = dp.RCC.constrain();
    // Now we have access to the RCC's registers. The GPIOC can be enabled in
    // RCC_APB2ENR (Prog. Ref. Manual 8.3.7), therefore we must pass this
    // register to the `split` function.
    
    let mut gpioc = dp.GPIOC.split(&mut rcc.apb2);
    
    // This gives us an exclusive handle to the GPIOC peripheral. To get the
    // handle to a single pin, we need to configure the pin first. Pin C13
    // is usually connected to the Bluepills onboard LED.
    
    let _led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);

    // Now we need a delay object. The delay is of course depending on the clock
    // frequency of the microcontroller, so we need to fix the frequency
    // first. The system frequency is set via the FLASH_ACR register, so we
    // need to get a handle to the FLASH peripheral first:
    let mut flash = dp.FLASH.constrain();
    
    // Now we can set the controllers frequency to 8 MHz:
    //let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let clocks = rcc.cfgr.sysclk(8.mhz()).freeze(&mut flash.acr);
    // let clocks = rcc
    //     .cfgr
    //     //.use_hse(8.mhz())
    //     //.sysclk(56.mhz())
    //     //.pclk1(28.mhz())
    //     .adcclk(1.khz())
    //     .freeze(&mut flash.acr);

    //hprintln!("adc freq: {}", clocks.adcclk().0).unwrap();
    // target halted due to debug-request, current mode: Thread 
    // xPSR: 0x01000000 pc: 0x08000130 msp: 0x20005000, semihosting
    // adc freq: 1000000

    // Setup ADC
    let mut adc1 = adc::Adc::adc1(dp.ADC1, &mut rcc.apb2, clocks);

    // Setup GPIOB
    let mut gpiob = dp.GPIOB.split(&mut rcc.apb2);
    let mut gpioa = dp.GPIOA.split(&mut rcc.apb2);

    // configure pa0 as an analog input
    let mut ch0 = gpioa.pa0.into_analog(&mut gpioa.crl);

    // The `clocks` handle ensures that the clocks are now configured and gives
    // the `Delay::new` function access to the configured frequency. With
    // this information it can later calculate how many cycles it has to
    // wait. The function also consumes the System Timer peripheral, so that no
    // other function can access it. Otherwise the timer could be reset during a
    // delay.
    let mut delay = Delay::new(cp.SYST, clocks);

    // Prepare the alternate function I/O registers
    let mut afio = dp.AFIO.constrain(&mut rcc.apb2);



    // USART1



    let tx = gpiob.pb6.into_alternate_push_pull(&mut gpiob.crl);
    let rx = gpiob.pb7;

    // Set up the usart device. Taks ownership over the USART register and tx/rx pins. The rest of
    // the registers are used to enable and configure the device.
    let mut serial = Serial::usart1(
        dp.USART1,
        (tx, rx),
        &mut afio.mapr,
        Config::default().baudrate(9600.bps()),
        clocks,
        &mut rcc.apb2,
    );

    let mut _nprint = |x: u32| {
        let mut xb: u32 = x;

        let mut c = 0;
        let mut buf: [u32; 10] = [0; 10];
        loop {
            let m = xb % 10;
            buf[c] = m;
            xb = xb / 10;
            if xb == 0 {
                break;
            }
            c = c + 1;        
        };
        let mut n_init = false;
        for x in buf.iter().rev() {
            if x != &0u32 {
                n_init = true;
            }
            if n_init {
                block!(serial.write(0x30 + (*x as u8))).ok();
            }
        }
        block!(serial.write(0x0A)).ok();
    };




    // Display



    // SPI1
    let sck = gpioa.pa5.into_alternate_push_pull(&mut gpioa.crl);
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7.into_alternate_push_pull(&mut gpioa.crl);


    let mut rst = gpiob.pb0.into_push_pull_output(&mut gpiob.crl);
    let dc = gpiob.pb1.into_push_pull_output(&mut gpiob.crl);

    let spi = Spi::spi1(
        dp.SPI1,
        (sck, miso, mosi),
        &mut afio.mapr,
        Mode {
            polarity: Polarity::IdleLow,
            phase: Phase::CaptureOnFirstTransition,
        },
        8.mhz(),
        clocks,
        &mut rcc.apb2,
    );

    let interface = display_interface_spi::SPIInterfaceNoCS::new(spi, dc);
    let mut disp: GraphicsMode<_> = Builder::new().connect(interface).into();


    disp.reset(&mut rst, &mut delay).unwrap();
    disp.init().unwrap();

    const N: usize = 1024;

    let mut samples: [f32; N] = [0.0; N];
    let mut samplesb: [f32; N] = [0.0; N];

    let _amplitudes: [u32; 128] = [0; 128];

    let mut window: [f32; N] = [0.0; N];
    
    for i in 0..N {
        let sin1 = F32Ext::sin(3.142*(i as f32)/(N as f32));
        window[i] = sin1 * sin1;
    }


    loop {
        for i in 0..64 {
            for j in 0..128 {
                disp.set_pixel(j, i, 0);
            } 
        }

        for i in 0..N {
            delay.delay_us(256u16);
            let data: u16 = adc1.read(&mut ch0).unwrap();

            //samples[i as usize] = (data/30 - 20) as f32;
            
            samples[i as usize] = (data as f32) * window[i];
        }

        for i in 0..N {
            samples[i] = samples[i] * 0.8 + samplesb[i] * 0.2;
            samplesb[i] = samples[i];
        }
        
        let spectrum = microfft::real::rfft_1024(&mut samples);

        let fl = 60;
        let fh = 128;
        let ampl_max = spectrum[fl..fh].iter()
            .map(|x| x.norm_sqr() as f32)
            .fold(0., |a, c| {
                if c > a {
                    return c;
                }
                a
            });
        

        //nprint(ampl_max as u32);

        for i in fl..fh {
            let ampl_n = spectrum[i as usize].norm_sqr() as f32;
            let ampl = (64.*(ampl_n/(ampl_max))) as u32;
            //let ampl = 64 * F32Ext::log10(ampl_n/(ampl_max)) as u32;
            for j in 0..ampl {
                if j < 64 {
                    disp.set_pixel(i as u32, 64-j, 1);
                }
            }
        }
        disp.flush().unwrap();
    }
}