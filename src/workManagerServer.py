#!/usr/bin/env python3
import pika
import json
import time

import amqpurl # A file that contains "url = '$URL'" for your amqp account


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    channel.basic_publish(exchange='', routing_key='commands', body='configure i experiments mn 1 exit')
    channel.basic_publish(exchange='', routing_key='commands', body='configure i experiments mn 1 exit')
    channel.basic_publish(exchange='', routing_key='commands', body='configure i experiments mn 1 exit')
    channel.basic_publish(exchange='', routing_key='commands', body='configure i experiments mn 1 exit')


