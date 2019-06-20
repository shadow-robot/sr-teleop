# cyberglove_trajectory

This package contains source code for interacting with the Cyberglove and sending trajectory control data to the hand.

## Mocking

In order to mock the cyberglove and send data to the hand without actual hardware, run:

```sh
rosrun cyberglove_trajectory cyberglove_mock.py
```

This will send a signal to the hand making it alternate between pack and open poses every 3 seconds.
