#!/usr/bin/env python3
import pika
import json

import amqpurl  # A file that contains "url = '$URL'" for your amqp account


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    while True:

        print("WorkManagerServer menu:\n")
        print("  evo - run evolutionary experiments")
        print("  rand - run random search experiments")
        print("  file - run jobs from file")
        print()
        print("  purge - purge command queue")
        print("  exit - exit the program")
        print()

        choice = input(">")

        if choice == "exit" or choice == "":
            break
        elif choice == "purge":
            channel.queue_purge(queue="jobs")
            print("  Job queue purged\n")
        elif choice == "evo":
            experimentNumber = int(input("  How many experiments do you want to run? >"))
            generations = int(input("  How many generations? >"))
            individuals = int(input("  How many individuals? >"))
            for i in range(experimentNumber):
                channel.basic_publish(exchange='', routing_key='jobs', body='{{"command": "experiments mn 1 exit", "settings": {{"generations": {}, "individuals": {}}}}}'.format(generations, individuals))
            print("  Sent {} evolutionary search jobs to the queue.\n".format(experimentNumber))
        elif choice == "rand":
            experimentNumber = int(input("  How many experiments do you want to run? >"))
            generations = int(input("  How many generations? >"))
            individuals = int(input("  How many individuals? >"))
            for i in range(experimentNumber):
                channel.basic_publish(exchange='', routing_key='jobs', body='{{"command": "experiments ra 1 exit", "settings": {{"generations": {}, "individuals": {}}}}}'.format(generations, individuals))
            print("  Sent {} random search jobs to the queue.\n".format(experimentNumber))
        elif choice == "file":
            print("    read - read jobs from file")
            print("    write - write an example file to disk to edit")

            choice2 = input("    Command >")

            if choice2 == "read":
                filename = input("        Filename? >")
                experimentNumber = int(input("        How many of each experiment do you want to run? >"))

                with open(filename) as json_file:
                    data = json.load(json_file)
                for i in range(experimentNumber):
                    for experimentLine in data:
                        channel.basic_publish(exchange='', routing_key='jobs', body=json.dumps(experimentLine))

                print("        Jobs sent successfully\n")
            elif choice2 == "write":
                filename = input("        Filename? >")
                file = open(filename, "w")

                file.write("[\n  {\n    \"command\": \"firstCommand\",\n    \"settings\": {\n      \"generations\": 8, \n      \"individuals\": 8\n    }\n  },\n  {\n    \"command\": \"secondCommand\",\n    \"settings\": {\n      \"generations\": 8, \n      \"individuals\": 8\n    }\n  }\n]")
                file.close()

            else:
                print("Unknown command!\n")
        else:
            print("Unknown command\n")
