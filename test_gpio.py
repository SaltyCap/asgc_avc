import lgpio
handle = lgpio.gpiochip_open(4)
print("Chip opened successfully, handle:", handle)
try:
    lgpio.gpio_claim_output(handle, 11, 0)
    print("Claimed PIN 11 output")
    lgpio.gpio_write(handle, 11, 1)
    print("Wrote 1 to PIN 11")
    lgpio.gpio_write(handle, 11, 0)
    print("Wrote 0 to PIN 11")
except Exception as e:
    print("Failed to claim/write output:", e)
lgpio.gpiochip_close(handle)
