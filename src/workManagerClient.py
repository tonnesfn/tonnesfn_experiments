#!/usr/bin/env python3
import pika
import subprocess
import platform
import time
import json
import string
import os
import sys
from subprocess import PIPE, Popen
from threading import Thread
from queue import Queue, Empty

import amqpurl # A file that contains "url = '$URL'" for your amqp account


def enqueue_output(out, queue):
    for line in iter(out.readline, b''):
        queue.put(line)
    out.close()


def callback(ch, method, properties, body):
    print("Received message: {}".format(body), file=sys.stderr)

    # Extract message and settings:
    message = json.loads(body.decode('utf-8'))

    # Write settings file:
    print(os.path.join(os.path.expanduser('~'),'catkin_ws/src/tonnesfn_experiments/src/evoSettings.h'), file=sys.stderr)
    with open(os.path.join(os.path.expanduser('~'),'catkin_ws/src/tonnesfn_experiments/src/evoSettings.h'), "w") as evo_settings_file:
        evo_settings_file.write("const int popSize =  {};\nconst int generations = {};\n".format(message['settings']['individuals'], message['settings']['generations']))

    if (message['node'] != 'expGui') and (message['node'] != 'mapGui'):
        print("  Invalid node: {}".format(message['node']), file=sys.stderr)
        channel.basic_publish(exchange='', routing_key='results', body='Invalid node: {}'.format(message['node']))
        return

    # Building the project:
    print("Building tonnesfn_experiments:", file=sys.stderr)
    result = subprocess.run('cd ~/catkin_ws && source devel/setup.bash && catkin build tonnesfn_experiments', stdout=subprocess.PIPE, shell=True, executable='/bin/bash')
    result_str = result.stdout.decode('utf-8')

    if "packages succeeded!" in result_str:
        print("  Compilation successful", file=sys.stderr)
    else:
        print("  Compilation failed", file=sys.stderr)
        channel.basic_publish(exchange='', routing_key='results', body='Compilation failed: {}'.format(result_str))
        return

    # Executing the program:
    processCommand = ['rosrun', 'tonnesfn_experiments', message['node']]
    processCommand.extend(message['command'].split())
    console_process = subprocess.Popen(processCommand, stdout=subprocess.PIPE, bufsize=0, close_fds=True)

    q = Queue()
    t = Thread(target=enqueue_output, args=(console_process.stdout, q))
    t.daemon = True  # thread dies with the program
    t.start()

    console_output_str = ""

    try:

        while console_process.poll() is None:

            try:
                line = q.get_nowait()
            except Empty:
                pass
            else:
                print(line.decode('utf-8'), end='', flush=True, file=sys.stderr)
                console_output_str += ''.join(filter(lambda x: x in string.printable, line.decode('utf-8')))
                time.sleep(0.1)

            connection.process_data_events()

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Killing process!", file=sys.stderr)
        console_process.kill()
        exit()

    time.sleep(3)

    # Empty the rest of the queue
    while True:
        try:
            line = q.get_nowait()
        except Empty:
            break
        else:
            print(line.decode('utf-8'), end='', flush=True, file=sys.stderr)
            console_output_str += ''.join(filter(lambda x: x in string.printable, line.decode('utf-8')))
            time.sleep(0.1)

    if "Error while calling GaitRecording" in console_output_str:
        print("Returncode: {}".format(console_process.returncode))
        channel.basic_nack(delivery_tag=method.delivery_tag)
        exit(-1)


    #console_output_str = console_process.stdout.read().decode('utf-8')
    #console_output_str = ''.join(filter(lambda x: x in string.printable, console_output_str))

    if "ABORT" in console_output_str:
        channel.basic_nack(delivery_tag=method.delivery_tag)
        print("Experiment aborted.", file=sys.stderr)
        return

    returnMessage = '{\n'
    returnMessage += "  \"node\": \"{}\",\n".format(platform.node())
    returnMessage += "  \"load\": [{}, {}, {}],\n".format(os.getloadavg()[0], os.getloadavg()[1], os.getloadavg()[2])
    returnMessage += "  \"consoleOutput\": \n"
    returnMessage += "    \"{}\"\n".format(console_output_str.replace("\n", "\\n").replace("\"", "\\\""))

    returnMessage += ","

    logFiles = []

    for line in console_output_str.splitlines():
        if ".json" in line:
            logFiles.append(line.strip())

    for file in logFiles:
        with open(file, 'r') as logFile:
            returnMessage += '\n"logFile" : {{\n  "name": \"{}\",\n  "contents": {}\n}}'.format(file, logFile.read())

    returnMessage += "\n}"

    channel.basic_publish(exchange='', routing_key='results', body=returnMessage)

    channel.basic_ack(delivery_tag=method.delivery_tag)

    print("Done!", file=sys.stderr)


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)

    channel = connection.channel()
    channel.basic_qos(prefetch_count=1)

    channel.queue_declare(queue='jobs')

    channel.basic_consume(callback, queue='jobs')

    channel.start_consuming()
