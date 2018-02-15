from datetime import datetime
import time


class EventDrivenChatBot:

    def __init__(self):
        # accepted_messages maps incoming messages to
        # list of callback functions
        self.accepted_messages = {}

        # time of instantiation
        self.birth_time = datetime.now()

        # "state" variable for handling responds to age
        self.has_been_asked_age = False

        # "registering" all callbacks
        self.register_callback("hi",
                               self.respond_to_greeting)
        self.register_callback("bye",
                               self.respond_to_departure)
        self.register_callback("age?",
                               self.handle_age_request)

    def register_callback(self, message, callback):
        """
        Registers a callback to a message.
        """
        if message not in self.accepted_messages:
            self.accepted_messages[message] = []
        self.accepted_messages[message].append(callback)

    def respond_to_greeting(self):
        print("Hello!")

    def respond_to_departure(self):
        print("Nice chatting with you!")

    def respond_to_age_request(self):
        age = datetime.now() - self.birth_time
        print("I am", age.seconds, "seconds old.")

    def respond_to_age_request_detailed(self):
        age = datetime.now() - self.birth_time
        micros = age.microseconds
        print("Technically, I'm", age.seconds, "seconds and",
              micros, "microseconds old")

    def handle_age_request(self):
        if self.has_been_asked_age:
            self.respond_to_age_request_detailed()
        else:
            self.has_been_asked_age = True
            self.respond_to_age_request()

    def handle_message(self, message):
        default_respond = [lambda: print("Sorry, I don't understand...")]
        callbacks = self.accepted_messages.get(message, default_respond)
        for callback in callbacks:
            callback()


if __name__== '__main__':
    bot = EventDrivenChatBot()
    bot.handle_message("hi")
    bot.handle_message("Yo~")
    time.sleep(2.345)
    bot.handle_message("age?")
    print("Let me ask you again...")
    bot.handle_message("age?")
    bot.handle_message("bye")
