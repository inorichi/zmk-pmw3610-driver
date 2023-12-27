PMW3610 driver implementation for ZMK with at least Zephyr 3.5

This work is based on [ufan's implementation](https://github.com/ufan/zmk/tree/support-trackpad) of the driver.

## Installation

Include this project on your ZMK's west manifest in `app/west.yml`:

```yml
manifest:
  remotes:
    - name: zephyrproject-rtos
      url-base: https://github.com/zephyrproject-rtos
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: petejohanson
      url-base: https://github.com/petejohanson
    - name: inorichi
      url-base: https://github.com/inorichi
  projects:
    - name: zephyr
      remote: petejohanson
      revision: v3.5.0+zmk-fixes
      clone-depth: 1
      import:
        name-blocklist:
          - ci-tools
          - hal_altera
          - hal_cypress
          - hal_infineon
          - hal_microchip
          - hal_nxp
          - hal_openisa
          - hal_silabs
          - hal_xtensa
          - hal_st
          - hal_ti
          - loramac-node
          - mcuboot
          - mcumgr
          - net-tools
          - openthread
          - edtt
          - trusted-firmware-m
    - name: zmk-pmw3610-driver
      remote: inorichi
      revision: main
  self:
    west-commands: scripts/west-commands.yml

```

Then on a terminal do `west update` to apply changes.

Now, update your `board.overlay` adding the necessary bits (update the pins for your board):

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };

    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 8)>,
                <NRF_PSEL(SPIM_MOSI, 0, 17)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

&spi0 {
    status = "okay";
	compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 20 GPIO_ACTIVE_LOW>;

	trackball: trackball@0 {
        status = "okay";
		compatible = "pixart,pmw3610";
		reg = <0>;
		spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 6 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        snipe-layers = <>; // optional indexes of snipe/precision layers
		scroll-layers = <>; // optional indexes of drag scroll layers
	};
};
```

Finally, enable the driver config in your `board.config` file (open the Kconfig file to find out all possible options):

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_MOUSE=y
CONFIG_PMW3610=y
```
