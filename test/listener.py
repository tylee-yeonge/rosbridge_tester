import roslibpy
import time

client = roslibpy.Ros(host='localhost', port=9090)
client.run()
# subscriber = roslibpy.Topic(client, 'rosbridge_test', 'std_msgs/String')
subscriber_state = roslibpy.Topic(client, 'rosbridge_tester/task_state', 'thira_task_msgs/TaskState')
subscriber_event = roslibpy.Topic(client, 'rosbridge_tester/task_event', 'thira_task_msgs/TaskEvent')
subscriber_state.subscribe(lambda message: print(message))
subscriber_event.subscribe(lambda message: print(message))

service = roslibpy.Service(client, '/rosbridge_tester/update', 'std_srvs/Empty')
request = roslibpy.ServiceRequest()



while client.is_connected:
    print('Calling service...')
    result = service.call(request)
    print('Service response: {}'.format(result))
    time.sleep(1)


try:
    while True:
        pass

except KeyboardInterrupt:
    client.terminate()