def topic_name(function):
    return function.__qualname__.replace('.', '/').lower()