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
        print("  exit - exit the program")
        print();

        choice = input(">")

        if choice == "exit":
            break;
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

        else:
            print("Unknown")
