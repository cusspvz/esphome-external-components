# Crow Runner ESPHome External Component
## @cusspvz/esphome-crow-runner

This repository holds the external components that I've developed for personal use.

## Components

### [Crow Runner](./components/crow-runner) Alarms

This component connects to the Crow Runner 4/8 alarm and integrates all of its functionalities within Home Assistant. This allows you to replace or increase virtually the number of keypads within your house, or even control your alarm system without relying on the Crow's IP Module.

## Development

### Setup dev environment

```bash
pipenv install
```


### Compile the components

```bash
pipenv run esphome compile ./test.yml
```


### Compile and run

1. Connect a ESP8266 D1 Board

> you can use others as long as you make the necessary changes to the `test-device.yml` file

2. Compile and Upload to the locally connect
```bash
pipenv run esphome run ./test.yml
```

## Reverse engineering

### Message Format

- Has a `7e` Start and End boundary byte
- Message can have multiple sizes, depending on the message type
  - Zones update has 2 bytes
  - Entire status has 8 bytes
  - Zone status has 6 bytes

### Keypad messages

| Hex | Description |
|---|-----------|
| `8b 00 80` | Keypad 1 |
| `8b 00 40` | Keypad 2 |
| `8b 00 c0` | Keypad 3 |
| `8b 00 20` | Keypad 4 |
| `8b 00 a0` | Keypad 5 |
| `8b 00 60` | Keypad 6 |
| `8b 00 e0` | Keypad 7 |
| `8b 00 10` | Keypad 8 |
| `8b 00 90` | Keypad 9 |
| `8b 00 00` | Keypad 0 |
| `8b 00 88` | Enter |
| `8b 00 30` | Panic |
| `8b 00 d0` | Memory |
| `8b 00 b0` | Arm |
| `8b 00 70` | Stay |
| `8b 00 f0` | Bypass |
| `8b 00 08` | Program |

### Status Messages

| Hex | Description |
|---|-----------|
| `2a 80 00 f2 78 f0 80 30` | Keypad 1 |
| `2a 80 00 8a 78 f0 80 30` | Keypad 1 |
| `2a 80 00 4a 00 f0 80 30` | Keypad 1 |
| `2a 80 00 4a f0 f0 80 30` | Keypad 1 |
| `2a 80 00 4a 78 f0 80 30` | Keypad 1 |
| `2a 80 00 4a b4 f0 80 30` | Keypad 1 |
| `2a 80 00 ca 00 f0 80 30` | Keypad 1 |
| `28 00 91 00 00 80 01` | Keypad 1 |
| `2a 80 00 ca f0 f0 80 30` | Keypad 1 |
| `28 00 55 00 00 80 01` | Keypad 1 |
| `2a 80 00 ca b4 f0 80 30` | Keypad 1 |
| `2a 80 00 2a 00 f0 80 30` | Keypad 1 |
| `0a 00` | Keypad 1 |
| `0a 20` | Keypad 1 |
| `0a e0` | Keypad 1 |
| `0a 60` | Keypad 1 |
| `a8 00 00 00 00 80` | Keypad 1 |
| `0a 60` | Keypad 1 |
| `28 00 80 00 00 80 01` | Keypad 1 |
| `2a 80 00 2a 78 f0 80 30` | Keypad 1 |
| `0a 00` | Keypad 1 |
| `08 00 00 03 00 00` | Keypad 1 |
| `a8 00 00 00 00 80` | Keypad 1 |
| `28 00 80 00 00 00 01` | Keypad 1 |



## License

[MIT](./LICENSE)

Copyright 2024 José Moreira <github.com/cusspvz>
