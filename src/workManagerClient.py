#!/usr/bin/env python3
import pika
import subprocess
import platform
import time
import json
import string
import os

import amqpurl # A file that contains "url = '$URL'" for your amqp account


def callback(ch, method, properties, body):
    print("Received message: {}".format(body))

    # Extract message and settings:
    message = json.loads(body.decode('utf-8'))

    # Write settings file:
    print(os.path.join(os.path.expanduser('~'),'catkin_ws/src/tonnesfn_experiments/src/evoSettings.h'))
    with open(os.path.join(os.path.expanduser('~'),'catkin_ws/src/tonnesfn_experiments/src/evoSettings.h'), "w") as evo_settings_file:
        evo_settings_file.write("const int popSize =  {};\nconst int generations = {};\n".format(message['settings']['individuals'], message['settings']['generations']))

    # Building the project:
    print("Building tonnesfn_experiments:")
    result = subprocess.run(['catkin', 'build', 'tonnesfn_experiments'], stdout=subprocess.PIPE)
    result_str = result.stdout.decode('utf-8')

    if "packages succeeded!" in result_str:
        print("  Compilation successful")
    else:
        print("  Compilation failed")
        channel.basic_publish(exchange='', routing_key='results', body='Compilation failed: {}'.format(result_str))
        return

    # Executing the program:
    processCommand = ['rosrun', 'tonnesfn_experiments', 'expGui']
    processCommand.extend(message['command'].split())
    console_process = subprocess.Popen(processCommand, stdout=subprocess.PIPE)

    try:

        while console_process.poll() is None:
            connection.process_data_events()
            time.sleep(1)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Killing process!")
        console_process.kill()
        exit()

    console_output_str = console_process.stdout.read().decode('utf-8')
    console_output_str = ''.join(filter(lambda x: x in string.printable, console_output_str))

    returnMessage = '{\n'
    returnMessage += "  \"node\": \"{}\",\n".format(platform.node())
    returnMessage += "  \"consoleOutput\": \n"
    returnMessage += "    \"{}\"\n".format(console_output_str.replace("\n", "\\n"))
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

    channel.basic_ack(delivery_tag = method.delivery_tag)

    print("Done!")


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)

    channel = connection.channel()
    channel.basic_qos(prefetch_count=1)

    channel.queue_declare(queue='jobs')

    channel.basic_consume(callback, queue='jobs')

    channel.start_consuming()
