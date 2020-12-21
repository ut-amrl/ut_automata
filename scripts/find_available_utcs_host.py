#!/usr/bin/env python3

import asyncio
import subprocess
import sys
from random import shuffle
from argparse import ArgumentParser

def get_machines(username):
    print("Getting list of CS hosts...", file=sys.stderr)
    try:
        output = subprocess.check_output([
            "ssh", 
            "-oStrictHostKeyChecking=no",
            f"{username}@linux.cs.utexas.edu", 
            "cshosts pub"
        ], stderr=subprocess.DEVNULL).decode("utf-8")
        machines = output.splitlines()
        shuffle(machines)
        return machines
    except subprocess.CalledProcessError:
        print("Could not get list of CS hosts. Make sure you have added your SSH key to this shell session.")
        exit(0)

async def test_command(username, machine, command, time_wait):
    proc = await asyncio.create_subprocess_exec(
        "ssh", 
        "-oStrictHostKeyChecking=no",
        f"{username}@{machine}.cs.utexas.edu", 
        command,
        stdout=asyncio.subprocess.DEVNULL,
        stderr=asyncio.subprocess.DEVNULL
    )
    try:
        exit_code = await asyncio.wait_for(proc.wait(), time_wait)
        if exit_code == 0:
            return (machine, True)
        return (machine, False)
    except asyncio.TimeoutError:
        proc.terminate()
        return (machine, False)

async def test_machines(*, username, command, time_wait):
    futures = [test_command(username, machine, command, time_wait) for machine in get_machines(username)]
    print("Checking hosts...", file=sys.stderr)
    for future in asyncio.as_completed(futures):
        yield await future

async def search(*, username, command, time_wait):
    async for machine, status in test_machines(username=username, command=command, time_wait=time_wait):
        if not status:
            continue
        print(machine)

async def connect(*, username, command, time_wait):
    async for machine, status in test_machines(username=username, command=command, time_wait=time_wait):
        if not status:
            continue
        subprocess.run(["ssh", f"{username}@{machine}.cs.utexas.edu"])
        return
    print("Could not find unused machine with ROS.", file=sys.stderr)

if __name__ == "__main__":
    ap = ArgumentParser("findros", description="Automatically connect to an unused lab machine with ROS")
    ap.add_argument("username", help="CS lab machine username")
    ap.add_argument("-t", "--time_wait", nargs="?", const = 30, type=int, help="ssh waiting time before timing out")
    ap.add_argument("-s", "--search", dest="search", action="store_true", help="List machines but don't connect")
    ap.add_argument("-i", "--in-use", dest="inuse", action="store_true", help="Find machines with ROS that *ARE* in use")
    ns = ap.parse_args(sys.argv[1:])

    if ns.inuse:
        command = "ls /opt/ros && pgrep websocket"
    else:
        command = "ls /opt/ros && ! pgrep websocket"

    try:
        if ns.search:
            loop = asyncio.get_event_loop()
            loop.run_until_complete(search(username=ns.username, command=command, time_wait=ns.time_wait))
        else:   
            loop = asyncio.get_event_loop()
            loop.run_until_complete(connect(username=ns.username, command=command, time_wait=ns.time_wait))
    except KeyboardInterrupt:
        pass
