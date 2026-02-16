# Простой OPC UA клиент для записи значений в переменные сервера.

import asyncio

from asyncua import Client, ua


async def main():
    url = "opc.tcp://192.168.88.100:4840/freeopcua/server/"

    async def browse_recursive(node, depth=0):
        name = await node.read_browse_name()
        print("  " * depth + f"{name.Name} ({node.nodeid})")

        for child in await node.get_children():
            await browse_recursive(child, depth + 1)

    async with Client(url=url) as client:
        root = client.get_root_node()
        print(f"Корневой узел: {root}")

        # objects = client.get_objects_node()
        # await browse_recursive(objects)

        node = client.get_node("ns=4;s=|var|HCFA-PLC.Application.OPC.BladeStoppers.Stopper2.qCmd")
        value = await node.read_value()
        print('BladeStoppers.Stopper2.qCmd: ', value)

        await node.set_value(ua.DataValue(ua.Variant(1, ua.VariantType.Int16)))
        print('BladeStoppers.Stopper2.qCmd: ', value)


if __name__ == "__main__":
    asyncio.run(main())
