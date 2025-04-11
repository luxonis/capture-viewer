import multiprocessing
import multiprocessing.pool
 
def timeout(seconds=10):
    """
    Raise TimeoutError after given number of seconds if the decorated function does not return first.
    
    source: https://anonbadger.wordpress.com/2018/12/15/python-signal-handlers-and-exceptions/
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            pool = multiprocessing.pool.ThreadPool(processes=1)
            results = pool.apply_async(func, args, kwargs)
            pool.close()
            try:
                return results.get(seconds)
            except multiprocessing.TimeoutError:
                raise TimeoutError('Timeout expired after: %s' % seconds)
            finally:
                pool.terminate()
        return wrapper
    return decorator
