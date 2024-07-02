# @cusspvz/esphome-external-components

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


## License

[MIT](./LICENSE)

Copyright 2024 José Moreira <github.com/cusspvz>
