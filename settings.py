import logging
import os


LOG_FILEPATH = 'logs.log'
try:
    os.remove(LOG_FILEPATH)
except:
    pass

fh = logging.FileHandler(LOG_FILEPATH, encoding='utf-8')
sh = logging.StreamHandler()

logging.basicConfig(level=logging.INFO, handlers=[fh, sh])