#!/usr/bin/env python3
import pika

import amqpurl  # A file that contains "url = '$URL'" for your amqp account


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    while True:

        print("WorkManagerServer menu:\n")
        print("  evo - run evolutionary experiments")
        print("  rand - run random search experiments")
        print("  file - run commands from file")
        print()
        print("  purge - purge command queue")
        print("  exit - exit the program")
        print();

        choice = input(">")

        if choice == "exit" or choice == "":
            break;
        elif choice == "purge":
            channel.queue_purge(queue="commands")
            print("  Commands queue purged\n")
        elif choice == "evo":
            experimentNumber = int(input("  How many experiments do you want to run? >"))
            for i in range(experimentNumber):
                channel.basic_publish(exchange='', routing_key='commands', body='experiments mn 1 exit')
            print("  Sent {} evolutionary search jobs to the queue.\n".format(experimentNumber))
        elif choice == "rand":
            experimentNumber = int(input("  How many experiments do you want to run? >"))
            for i in range(experimentNumber):
                channel.basic_publish(exchange='', routing_key='commands', body='experiments ra 1 exit')
            print("  Sent {} random search jobs to the queue.\n".format(experimentNumber))
        elif choice == "file":
            print("    This command reads newline-separated commands from file and distributes these to the workers")
            filename = input("      Filename? >")
            experimentNumber = int(input("      How many of each experiment do you want to run? >"))

            with open(filename) as f:
                content = f.readlines()

            content = [x.strip() for x in content]

            for i in range(experimentNumber):
                for line in content:
                    channel.basic_publish(exchange='', routing_key='commands', body=line)

            print("      Commands sent successfully\n")
        else:
            print("Unknown command\n")
