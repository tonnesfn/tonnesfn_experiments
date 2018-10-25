#!/usr/bin/env python3
import pika
import json
import time

import amqpurl # A file that contains "url = '$URL'" for your amqp account


def callback(ch, method, properties, body):
    try:
        d = json.loads(body)

        logFileName = d['node'] + '_' + d['logFile']['name'].split('/')[-1]

        print("Received valid JSON results from {}, saving to: {}".format(d['node'], logFileName))

        file = open(logFileName, "w")
        file.write(json.dumps(d['logFile']['contents'], indent=4, sort_keys=True))
        file.close()
    except ValueError:

        filename = time.strftime("error_%Y%m%d%H%M%S.txt", time.localtime())

        file = open(filename, "w")
        file.write(body.decode("utf-8"))
        file.close()

        print("Received invalid JSON message, saved to ({})!".format(filename))

    channel.basic_ack(delivery_tag=method.delivery_tag)


if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()
    channel.basic_qos(prefetch_count=1)

    channel.queue_declare(queue='results')

    channel.basic_consume(callback, queue='results')

    channel.start_consuming()
