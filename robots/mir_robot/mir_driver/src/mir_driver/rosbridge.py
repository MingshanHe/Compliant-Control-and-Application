import websocket
import threading

import json
import traceback
import time

import string
import random


class RosbridgeSetup:
    def __init__(self, host, port):
        self.callbacks = {}
        self.service_callbacks = {}
        self.resp = None
        self.connection = RosbridgeWSConnection(host, port)
        self.connection.registerCallback(self.onMessageReceived)

    def publish(self, topic, obj):
        pub = {"op": "publish", "topic": topic, "msg": obj}
        self.send(pub)

    def subscribe(self, topic, callback, throttle_rate=-1):
        if self.addCallback(topic, callback):
            sub = {"op": "subscribe", "topic": topic}
            if throttle_rate > 0:
                sub['throttle_rate'] = throttle_rate

            self.send(sub)

    def unhook(self, callback):
        keys_for_deletion = []
        for key, values in self.callbacks.items():
            for value in values:
                if callback == value:
                    print("Found!")
                    values.remove(value)
                    if len(values) == 0:
                        keys_for_deletion.append(key)

        for key in keys_for_deletion:
            self.unsubscribe(key)
            self.callbacks.pop(key)

    def unsubscribe(self, topic):
        unsub = {"op": "unsubscribe", "topic": topic}
        self.send(unsub)

    def callService(self, serviceName, callback=None, msg=None):
        id = self.generate_id()
        call = {"op": "call_service", "id": id, "service": serviceName}
        if msg != None:
            call['args'] = msg

        if callback == None:
            self.resp = None

            def internalCB(msg):
                self.resp = msg
                return None

            self.addServiceCallback(id, internalCB)
            self.send(call)

            while self.resp == None:
                time.sleep(0.01)

            return self.resp

        self.addServiceCallback(id, callback)
        self.send(call)
        return None

    def send(self, obj):
        try:
            self.connection.sendString(json.dumps(obj))
        except:
            traceback.print_exc()
            raise

    def generate_id(self, chars=16):
        return ''.join(random.SystemRandom().choice(string.ascii_letters + string.digits) for _ in range(chars))

    def addServiceCallback(self, id, callback):
        self.service_callbacks[id] = callback

    def addCallback(self, topic, callback):
        if topic in self.callbacks:
            self.callbacks[topic].append(callback)
            return False

        self.callbacks[topic] = [callback]
        return True

    def is_connected(self):
        return self.connection.connected

    def is_errored(self):
        return self.connection.errored

    def onMessageReceived(self, message):
        try:
            # Load the string into a JSON object
            obj = json.loads(message)
            # print "Received: ", obj

            if 'op' in obj:
                option = obj['op']
                if option == "publish":  # A message from a topic we have subscribed to..
                    topic = obj["topic"]
                    msg = obj["msg"]
                    if topic in self.callbacks:
                        for callback in self.callbacks[topic]:
                            try:
                                callback(msg)
                            except:
                                print("exception on callback", callback, "from", topic)
                                traceback.print_exc()
                                raise
                elif option == "service_response":
                    if "id" in obj:
                        id = obj["id"]
                        values = obj["values"]
                        if id in self.service_callbacks:
                            try:
                                # print 'id:', id, 'func:', self.service_callbacks[id]
                                self.service_callbacks[id](values)
                            except:
                                print("exception on callback ID:", id)
                                traceback.print_exc()
                                raise
                    else:
                        print("Missing ID!")
                else:
                    print("Recieved unknown option - it was: ", option)
            else:
                print("No OP key!")
        except:
            print("exception in onMessageReceived")
            print("message", message)
            traceback.print_exc()
            raise


class RosbridgeWSConnection:
    def __init__(self, host, port):
        self.ws = websocket.WebSocketApp(
            ("ws://%s:%d/" % (host, port)), on_message=self.on_message, on_error=self.on_error, on_close=self.on_close
        )
        self.ws.on_open = self.on_open
        self.run_thread = threading.Thread(target=self.run)
        self.run_thread.start()
        self.connected = False
        self.errored = False
        self.callbacks = []

    def on_open(self):
        print("### ROS bridge connected ###")
        self.connected = True

    def sendString(self, message):
        if not self.connected:
            print("Error: not connected, could not send message")
            # TODO: throw exception
        else:
            self.ws.send(message)

    def on_error(self, error):
        self.errored = True
        print("Error: %s" % error)

    def on_close(self):
        self.connected = False
        print("### ROS bridge closed ###")

    def run(self, *args):
        self.ws.run_forever()

    def on_message(self, message):
        # Call the handlers
        for callback in self.callbacks:
            callback(message)

    def registerCallback(self, callback):
        self.callbacks.append(callback)
