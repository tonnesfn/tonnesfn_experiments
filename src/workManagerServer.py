#!/usr/bin/env python3
import pika
import json

import amqpurl # A file that contains "url = '$URL'" for your amqp account

def callback(ch, method, properties, body):
    print("Received results\n")

    d = json.loads(body)

    logFileName = d['node'] + '_' + d['logFile']['name'].split('/')[-1]

    file = open(logFileName, "w")
    file.write(json.dumps(d['logFile']['contents'], indent=4, sort_keys=True))
    file.close()

if __name__ == '__main__':
    params = pika.URLParameters(amqpurl.url)
    connection = pika.BlockingConnection(params)
    channel = connection.channel()

    channel.basic_publish(exchange='', routing_key='hello', body='configure i experiments mn 1 exit')


    channel.queue_declare(queue='results')

    channel.basic_consume(callback, queue='results', no_ack=True)

    channel.start_consuming()