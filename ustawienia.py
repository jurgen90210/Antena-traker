class StepperController:
    def __init__(self):
        self.chip = lgpio.gpiochip_open(0)

        # --- TMC2209: PAN ---
        self.pan_step = 23
        self.pan_dir  = 24
        self.pan_en   = 18

        # --- TMC2209: TILT ---
        self.tilt_step = 26
        self.tilt_dir  = 20
        self.tilt_en   = 16

        # Ustaw jako wyjścia i wymuś stan niski
        for pin in [self.pan_step, self.pan_dir, self.pan_en,
                    self.tilt_step, self.tilt_dir, self.tilt_en]:
            lgpio.gpio_claim_output(self.chip, pin)
            lgpio.gpio_write(self.chip, pin, 0)

        # Wyłącz silniki (EN = 1)
        lgpio.gpio_write(self.chip, self.pan_en, 1)
        lgpio.gpio_write(self.chip, self.tilt_en, 1)

        # Enkoder tylko dla PAN
        self.encoder_pan = OpticalEncoderE2(self.chip, pin_a=17, pin_b=27, cpr=200)

        print("✅ StepperController gotowy – TMC2209 + E2(PAN)")
