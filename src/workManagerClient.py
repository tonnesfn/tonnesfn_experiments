#!/usr/bin/env python3
import pika
import amqpurl # A file that contains "url = '$URL'" for your amqp account


def callback(ch, method, properties, body):
    print("Received message: {}".format(body))


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.queue_declare(queue='hello')

    channel.basic_consume(callback, queue='hello', no_ack=True)

    channel.start_consuming()
