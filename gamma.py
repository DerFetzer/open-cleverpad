#!/usr/bin/env python3

gamma = 2.8  # Correction factor
max_in = 3
max_out = 30

for i in range(max_in + 1):
    print(str(int(pow(i / max_in, gamma) * max_out + 0.5)) + ", ")
