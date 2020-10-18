//! (TODO) Inter-Integrated Circuit (I2C) bus
use crate::pac::i2c0::con::{ADDR_SLAVE_WIDTH_A, SPEED_A};

use crate::bit_utils::{u32_bit_is_clear, u32_bit_is_set, u32_set_bit, u32_toggle_bit};
use crate::clock::Clocks;
use crate::pac::I2C0;
use crate::pac::I2C1;
use core::cmp;
use core::marker::PhantomData;

use crate::{
    fpioa::{
        functions::{I2C1_SCLK, I2C1_SDA},
        io_pins::{Io30, Io31},
    },
    time::Hertz,
};
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};

// use crate::{
//     gpio::{
//         gpioa::{PA6, PA7},
//         gpiob::{PB2, PB3},
//         gpiod::{PD0, PD1},
//         gpioe::{PE4, PE5},
//         AlternateFunction, Floating, OpenDrain, OutputMode, AF3,
//     },
//     hal::blocking::i2c::{Read, Write, WriteRead},
//     sysctl::{self, Clocks},

// };

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Bus error
    Bus,
    /// Arbitration loss
    Arbitration,

    /// Missing Data ACK
    DataAck,

    /// Missing Addrees ACK
    AdrAck,

    #[doc(hidden)]
    _Extensible,
}

// FIXME these should be "closed" traits
/// SCL pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SclPin<I2C> {}

/// SDA pin -- DO NOT IMPLEMENT THIS TRAIT
pub unsafe trait SdaPin<I2C> {}

unsafe impl SclPin<I2C1> for Io30<I2C1_SCLK> {}
unsafe impl SdaPin<I2C1> for Io31<I2C1_SDA> {}

/// I2C peripheral operating in master mode
pub struct I2c<I2C, PINS> {
    i2c: I2C,
    pins: PINS,
}

// macro_rules! busy_wait {
//     ($i2c:expr, $flag:ident, $op:ident) => {
//         // in 'release' builds, the time between setting the `run` bit and checking the `busy`
//         // bit is too short and the `busy` bit is not reliably set by the time you get there,
//         // it can take up to 8 clock cycles for the `run` to begin so this delay allows time
//         // for that hardware synchronization
//         delay(2);

//         loop {
//             let mcs = $i2c.mcs.read();

//             if mcs.error().bit_is_set() {
//                 if mcs.adrack().bit_is_set() {
//                     return Err(Error::AdrAck);
//                 } else if mcs.datack().bit_is_set() {
//                     return Err(Error::DataAck);
//                 }
//                 return Err(Error::Bus);
//             } else if mcs.arblst().bit_is_set() {
//                 return Err(Error::Arbitration);
//             } else if mcs.$flag().$op() {
//                 break;
//             } else {
//                 // try again
//             }
//         }
//     };
// }

macro_rules! hal {
    ($($I2CX:ident: ($powerDomain:ident, $i2cX:ident),)+) => {
        $(
            /// Extension trait that constrains I2C peripheral
            pub trait I2cExt<SCL, SDA>: Sized {
                /// Configures an I2C peripheral
                fn configure(self, pins: (SCL, SDA), clocks: &Clocks
                ) -> I2c<Self, (SCL, SDA)> where
                    SCL:  SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>;
            }

            impl<SCL, SDA> I2cExt<SCL, SDA> for $I2CX {
                fn configure(self, pins: (SCL, SDA), clocks: &Clocks
                ) -> I2c<I2C1, (SCL, SDA)> where
                    SCL:  SclPin<$I2CX>,
                    SDA: SdaPin<$I2CX>,
                {
                    let i2c = self;

                    let v_width = ADDR_SLAVE_WIDTH_A::B7;
                    let v_period_clk_cnt = 0;

                    i2c.enable.write(|w| w.enable().clear_bit());
                    i2c.con.write(|w| w.master_mode().bit(true)
                                  .slave_disable().bit(true)
                                  .restart_en().bit(true)
                                  .addr_slave_width().variant(v_width)
                                  .speed().variant(SPEED_A::FAST));

                    unsafe {
                        i2c.ss_scl_hcnt.write(|w| w.count().bits(v_period_clk_cnt));
                        i2c.ss_scl_lcnt.write(|w| w.count().bits(v_period_clk_cnt));
                    }
                    //                    i2c.tar.write(|w| w.address().bits(slave_address));
                    i2c.intr_mask.write(|w| w.rx_under().clear_bit().rx_over().clear_bit()
                                        .rx_full().clear_bit()
                                        .tx_over().clear_bit()
                                        .tx_empty().clear_bit()
                                        .rd_req().clear_bit()
                                        .tx_abrt().clear_bit()
                                        .rx_done().clear_bit()
                                        .activity().clear_bit()
                                        .stop_det().clear_bit()
                                        .start_det().clear_bit()
                                        .gen_call().clear_bit());
                    //                    i2c.intr_mask.write(|w| w.bits(0));
                    i2c.dma_cr.write(|w| w.rdmae().set_bit()
                                     .tdmae().set_bit());

                    unsafe {
                        i2c.dma_rdlr.write(|w| w.value().bits(0));
                        i2c.dma_tdlr.write(|w| w.value().bits(4));
                    }

                    i2c.enable.write(|w| w.enable().set_bit());

                    I2c { i2c, pins }
                }
            }

            impl<SCL, SDA> I2c<$I2CX, (SCL, SDA)> {
//                 /// Configures the I2C peripheral to work in master mode
//                 pub fn $i2cX<F>(
//                     i2c: $I2CX,
//                     pins: (SCL, SDA),
//                     freq: F,
// //                    clocks: &Clocks,
// //                    pc: &sysctl::PowerControl,
//                 ) -> Self where
//                     F: Into<Hertz>,
//                     SCL: SclPin<$I2CX>,
//                     SDA: SdaPin<$I2CX>,
//                 {
//                     let v_width = ADDR_SLAVE_WIDTH_A::B7;
//                     let v_period_clk_cnt = 0;

//                     i2c.enable.write(|w| w.enable().clear_bit());
//                     i2c.con.write(|w| w.master_mode().bit(true)
//                                   .slave_disable().bit(true)
//                                   .restart_en().bit(true)
//                                   .addr_slave_width().variant(v_width)
//                                   .speed().variant(SPEED_A::FAST));

//                     unsafe {
//                         i2c.ss_scl_hcnt.write(|w| w.count().bits(v_period_clk_cnt));
//                         i2c.ss_scl_lcnt.write(|w| w.count().bits(v_period_clk_cnt));
//                     }
// //                    i2c.tar.write(|w| w.address().bits(slave_address));
//                     i2c.intr_mask.write(|w| w.rx_under().clear_bit().rx_over().clear_bit()
//                                         .rx_full().clear_bit()
//                                         .tx_over().clear_bit()
//                                         .tx_empty().clear_bit()
//                                         .rd_req().clear_bit()
//                                         .tx_abrt().clear_bit()
//                                         .rx_done().clear_bit()
//                                         .activity().clear_bit()
//                                         .stop_det().clear_bit()
//                                         .start_det().clear_bit()
//                                         .gen_call().clear_bit());
// //                    i2c.intr_mask.write(|w| w.bits(0));
//                     i2c.dma_cr.write(|w| w.rdmae().set_bit()
//                                      .tdmae().set_bit());

//                     unsafe {
//                         i2c.dma_rdlr.write(|w| w.value().bits(0));
//                         i2c.dma_tdlr.write(|w| w.value().bits(4));
//                     }

//                     i2c.enable.write(|w| w.enable().set_bit());

//                     I2c { i2c, pins }

//                     // initial code:
//                     // sysctl::clock_enable(IF::CLK);
//                     // sysctl::clock_set_threshold(IF::DIV, 3);
//                     // sysctl::reset(IF::RESET);

//                     // let v_i2c_freq = sysctl::clock_get_freq(IF::CLK);
//                     // let v_period_clk_cnt = v_i2c_freq / i2c_clk / 2;
//                     // let v_period_clk_cnt: u16 = v_period_clk_cnt.try_into().unwrap();
//                     // let v_period_clk_cnt = cmp::max(v_period_clk_cnt, 1);

//                     // use i2c0::con::{ADDR_SLAVE_WIDTH_A,SPEED_A};
//                     // let v_width = match address_width {
//                     //     7 => ADDR_SLAVE_WIDTH_A::B7,
//                     //     10 => ADDR_SLAVE_WIDTH_A::B10,
//                     //     _ => panic!("unsupported address width"),
//                     // };
//                     // unsafe {
//                     //     self.i2c.enable.write(|w| w.bits(0));
//                     //     self.i2c.con.write(|w| w.master_mode().bit(true)
//                     //                          .slave_disable().bit(true)
//                     //                          .restart_en().bit(true)
//                     //                          .addr_slave_width().variant(v_width)
//                     //                          .speed().variant(SPEED_A::FAST));
//                     //     self.i2c.ss_scl_hcnt.write(|w| w.count().bits(v_period_clk_cnt));
//                     //     self.i2c.ss_scl_lcnt.write(|w| w.count().bits(v_period_clk_cnt));
//                     //     self.i2c.tar.write(|w| w.address().bits(slave_address));
//                     //     self.i2c.intr_mask.write(|w| w.bits(0));
//                     //     self.i2c.dma_cr.write(|w| w.bits(0x3));
//                     //     self.i2c.dma_rdlr.write(|w| w.bits(0));
//                     //     self.i2c.dma_tdlr.write(|w| w.bits(4));
//                     //     self.i2c.enable.write(|w| w.enable().bit(true));
//                     // }
//                 }

                /// Releases the I2C peripheral and associated pins
                pub fn free(self) -> ($I2CX, (SCL, SDA)) {
                    (self.i2c, self.pins)
                }
            }

            impl<PINS> Write for I2c<$I2CX, PINS> {
                type Error = Error;

                fn try_write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
                    unsafe {
                        let mut txi = 0;
                        let mut tx_left = bytes.len();

                        // Clear TX abort by reading from clear register
                        // Hopefully this is not optimized out
                        self.i2c.clr_tx_abrt.read().clr();

                        // Send all data that is left, handle errors that occur
                        while tx_left != 0 {
                            let fifo_len = 8 - (self.i2c.txflr.read().bits() as usize);
                            let fifo_len = cmp::min(tx_left, fifo_len);
                            for _ in 0..fifo_len {
                                self.i2c.data_cmd.write(|w| w.data().bits(bytes[txi]));
                                txi += 1;
                            }
                            if self.i2c.tx_abrt_source.read().bits() != 0 {
                                return Err(Error::Bus); // FIXME: This error is probably not the correct one.
                            }
                            tx_left -= fifo_len;
                        }

                        // Wait for TX succeed
                        while self.i2c.status.read().activity().bit() || !self.i2c.status.read().tfe().bit() {
                            // NOP
                        }

                        // Check for errors one last time
                        if self.i2c.tx_abrt_source.read().bits() != 0 {
                            return Err(Error::Bus); // FIXME: This error is probably not the correct one.
                        }
                    }
                    Ok(())

                    //origin code https://github.com/laanwj/k210-sdk-stuff/:
                    // unsafe {
                    //     let mut txi = 0;
                    //     let mut tx_left = send_buf.len();

                    //     // Clear TX abort by reading from clear register
                    //     // Hopefully this is not optimized out
                    //     self.i2c.clr_tx_abrt.read().bits();

                    //     // Send all data that is left, handle errors that occur
                    //     while tx_left != 0 {
                    //         let fifo_len = 8 - (self.i2c.txflr.read().bits() as usize);
                    //         let fifo_len = cmp::min(tx_left, fifo_len);
                    //         for _ in 0..fifo_len {
                    //             self.i2c.data_cmd.write(|w| w.data().bits(send_buf[txi]));
                    //             txi += 1;
                    //         }
                    //         if self.i2c.tx_abrt_source.read().bits() != 0 {
                    //             return Err(());
                    //         }
                    //         tx_left -= fifo_len;
                    //     }

                    //     // Wait for TX succeed
                    //     while self.i2c.status.read().activity().bit() || !self.i2c.status.read().tfe().bit() {
                    //         // NOP
                    //     }

                    //     // Check for errors one last time
                    //     if self.i2c.tx_abrt_source.read().bits() != 0 {
                    //         return Err(());
                    //     }
                    // }
                }
            }

            impl<PINS> Read for I2c<$I2CX, PINS> {
                type Error = Error;

                fn try_read(
                    &mut self,
                    addr: u8,
                    buffer: &mut [u8],
                ) -> Result<(), Error> {

                    let mut cmd_count = buffer.len();
                    let mut rx_left = buffer.len();
                    let mut rxi = 0;
                    while cmd_count != 0 || rx_left != 0 {
                        /* XXX this is a kind of strange construction, sanity check */
                        let fifo_len = self.i2c.rxflr.read().bits() as usize;
                        let fifo_len = cmp::min(rx_left, fifo_len);
                        for _ in 0..fifo_len {
                            buffer[rxi] = self.i2c.data_cmd.read().data().bits();
                            rxi += 1;
                        }
                        rx_left -= fifo_len;

                        /* send 0x100 for every byte that we want to receive */
                        let fifo_len = 8 - self.i2c.txflr.read().bits() as usize;
                        let fifo_len = cmp::min(cmd_count, fifo_len);
                        for _ in 0..fifo_len {
                            self.i2c.data_cmd.write(|w| w.cmd().bit(true));
                        }
                        if self.i2c.tx_abrt_source.read().bits() != 0 {
                            return Err(Error::Bus); // FIXME: This error is probably not the correct one.
                        }
                        cmd_count -= fifo_len;
                    }

                    Ok(())

                    // original code from https://github.com/laanwj/k210-sdk-stuff/:
                    // unsafe {
                    //     let mut txi = 0;
                    //     let mut tx_left = send_buf.len();
                    //     while tx_left != 0 {
                    //         let fifo_len = 8 - (self.i2c.txflr.read().bits() as usize);
                    //         let fifo_len = cmp::min(tx_left, fifo_len);
                    //         for _ in 0..fifo_len {
                    //             self.i2c.data_cmd.write(|w| w.data().bits(send_buf[txi]));
                    //             txi += 1;
                    //         }
                    //         if self.i2c.tx_abrt_source.read().bits() != 0 {
                    //             return Err(());
                    //         }
                    //         tx_left -= fifo_len;
                    //     }

                    //     let mut cmd_count = receive_buf.len();
                    //     let mut rx_left = receive_buf.len();
                    //     let mut rxi = 0;
                    //     while cmd_count != 0 || rx_left != 0 {
                    //         /* XXX this is a kind of strange construction, sanity check */
                    //         let fifo_len = self.i2c.rxflr.read().bits() as usize;
                    //         let fifo_len = cmp::min(rx_left, fifo_len);
                    //         for _ in 0..fifo_len {
                    //             receive_buf[rxi] = self.i2c.data_cmd.read().data().bits();
                    //             rxi += 1;
                    //         }
                    //         rx_left -= fifo_len;

                    //         /* send 0x100 for every byte that we want to receive */
                    //         let fifo_len = 8 - self.i2c.txflr.read().bits() as usize;
                    //         let fifo_len = cmp::min(cmd_count, fifo_len);
                    //         for _ in 0..fifo_len {
                    //             self.i2c.data_cmd.write(|w| w.cmd().bit(true));
                    //         }
                    //         if self.i2c.tx_abrt_source.read().bits() != 0 {
                    //             return Err(());
                    //         }
                    //         cmd_count -= fifo_len;
                    //     }
                    // }
                }
            }

            impl<PINS> WriteRead for I2c<$I2CX, PINS> {
                type Error = Error;

                fn try_write_read(
                    &mut self,
                    addr: u8,
                    bytes: &[u8],
                    buffer: &mut [u8],
                ) -> Result<(), Error> {

                    // let write_len = bytes.len();

                    // if buffer.len() == 0 {
                    //    return self.write(addr, bytes);
                    // }

                    // if bytes.len() == 0 {
                    //     return self.read(addr, buffer);
                    // }

                    // // Write Slave address and clear Receive bit
                    // self.i2c.msa.write(|w| unsafe {
                    //     w.sa().bits(addr)
                    // });

                    // // send first byte
                    // self.i2c.mdr.write(|w| unsafe {
                    //     w.data().bits(bytes[0])
                    // });

                    // busy_wait!(self.i2c, busbsy, bit_is_clear);

                    // self.i2c.mcs.write(|w| {
                    //     w.start().set_bit()
                    //         .run().set_bit()
                    // });

                    // busy_wait!(self.i2c, busy, bit_is_clear);

                    // for byte in (&bytes[1..write_len]).iter() {
                    //     self.i2c.mdr.write(|w| unsafe {
                    //         w.data().bits(*byte)
                    //     });

                    //     self.i2c.mcs.write(|w| {
                    //         w.run().set_bit()
                    //     });

                    //     busy_wait!(self.i2c, busy, bit_is_clear);
                    // }

                    // // Write Slave address and set Receive bit
                    // self.i2c.msa.write(|w| unsafe {
                    //     w.sa().bits(addr)
                    //         .rs().set_bit()
                    // });

                    // let recv_sz = buffer.len();

                    // if recv_sz == 1 {
                    //     // emit Repeated START and STOP for single receive
                    //     self.i2c.mcs.write(|w| {
                    //         w.run().set_bit()
                    //             .start().set_bit()
                    //             .stop().set_bit()
                    //     });

                    //     busy_wait!(self.i2c, busy, bit_is_clear);
                    //     buffer[0] = self.i2c.mdr.read().data().bits();
                    // } else {
                    //     // emit Repeated START
                    //     self.i2c.mcs.write(|w| {
                    //         w.run().set_bit()
                    //             .start().set_bit()
                    //             .ack().set_bit()
                    //     });

                    //     busy_wait!(self.i2c, busy, bit_is_clear);
                    //     buffer[0] = self.i2c.mdr.read().data().bits();

                    //     for byte in &mut buffer[1..recv_sz-1] {
                    //         self.i2c.mcs.write(|w| {
                    //             w.run().set_bit()
                    //                 .ack().set_bit()
                    //         });
                    //         busy_wait!(self.i2c, busy, bit_is_clear);
                    //         *byte = self.i2c.mdr.read().data().bits();
                    //     }

                    //     self.i2c.mcs.write(|w| {
                    //         w.run().set_bit()
                    //             .stop().set_bit()
                    //     });

                    //     busy_wait!(self.i2c, busy, bit_is_clear);
                    //     buffer[recv_sz-1] = self.i2c.mdr.read().data().bits();
                    // }

                    Ok(())
                }
            }
        )+
    }
}

hal! {
    I2C1: (I2c1, i2c1),
}
