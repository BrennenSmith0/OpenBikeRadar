"""
ble.py

Minimal BLE peripheral that advertises the Garmin Varia radar GATT
service/characteristic, so a Garmin Edge 840 can discover it the same
way it discovers a real RTL515 (Sensors > Add Sensor > Radar).

This is step one: prove discovery + connection work. It does not send
any real target data yet -- there's no notification loop wired to
frame_parser.py output here. Once the Edge 840 confirms it sees this
as a radar sensor, the next step is figuring out the Varia's actual
notification payload format and filling in update_value() calls
whenever a new RadarFrame comes in from the sensor pipeline.

Run directly on the Pi:

    python ble.py

Requires `bless` (BLE peripheral/server support -- Bleak alone is
central-only and can't advertise). On Linux this talks to BlueZ over
D-Bus, so bluetoothd must be running and not blocked by rfkill.
"""

from __future__ import annotations

import asyncio
import logging
import sys
import threading
from typing import Union

from bless import (
    BlessServer,
    BlessGATTCharacteristic,
    GATTCharacteristicProperties,
    GATTAttributePermissions,
)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Confirmed Varia RTL515 GATT identifiers.
RADAR_SERVICE_UUID = "6A4E3200-667B-11E3-949A-0800200C9A66"
RADAR_CHARACTERISTIC_UUID = "6A4E3202-667B-11E3-949A-0800200C9A66"

DEVICE_NAME = "OpenBikeRadar"

# bless's own examples use a threading.Event on macOS/Windows and an
# asyncio.Event on Linux, since those backends signal differently.
# We're Linux/BlueZ only right now, but keeping the same pattern costs
# nothing and matches upstream examples if this ever needs to run
# somewhere else during development.
stop_event: Union[asyncio.Event, threading.Event]
if sys.platform in ("darwin", "win32"):
    stop_event = threading.Event()
else:
    stop_event = asyncio.Event()


def on_read(characteristic: BlessGATTCharacteristic, **kwargs) -> bytearray:
    """Called if a central reads the characteristic directly."""
    logger.info("Central read characteristic: %s", characteristic.value)
    return characteristic.value


async def build_server(loop: asyncio.AbstractEventLoop) -> BlessServer:
    """Construct and configure the GATT server, without starting it."""

    server = BlessServer(name=DEVICE_NAME, loop=loop)
    server.read_request_func = on_read

    await server.add_new_service(RADAR_SERVICE_UUID)

    # Real Varia behavior is notify-driven: the Edge subscribes once
    # connected and receives updates, it doesn't poll. `read` is only
    # included here so a generic BLE scanner app can sanity-check the
    # characteristic exists while debugging.
    char_flags = (
        GATTCharacteristicProperties.read
        | GATTCharacteristicProperties.notify
    )
    permissions = GATTAttributePermissions.readable

    await server.add_new_characteristic(
        RADAR_SERVICE_UUID,
        RADAR_CHARACTERISTIC_UUID,
        char_flags,
        None,
        permissions,
    )

    return server


async def run() -> None:
    loop = asyncio.get_event_loop()
    server = await build_server(loop)

    await server.start()
    logger.info("Advertising as '%s'", DEVICE_NAME)
    logger.info("Service UUID:        %s", RADAR_SERVICE_UUID)
    logger.info("Characteristic UUID: %s", RADAR_CHARACTERISTIC_UUID)
    logger.info("On the Edge 840: Sensors > Add Sensor > Radar, and look for it there.")
    logger.info("Press Ctrl+C to stop.")

    try:
        while True:
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        logger.info("Stopping advertisement.")
        await server.stop()


if __name__ == "__main__":
    asyncio.run(run())