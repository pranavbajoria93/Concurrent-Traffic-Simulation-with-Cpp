#include <iostream>
#include <random>
#include "TrafficLight.h"

/* Implementation of class "MessageQueue" */
// method to wait for and receive new messages and pull them from the queue using move semantics.
template <typename T>
T MessageQueue<T>::receive()
{
    // modify deque w.r.t lock
    std::unique_lock<std::mutex> uLock(_mutex);
    _cond.wait(uLock, [this] { return !_queue.empty(); }); // pass unique lock to condition variable

    // deque last element from queue
    T message = std::move(_queue.back());
    _queue.pop_back();

    return message; // Return Value optimization makes sure, value is not copied
}

// add a new message to the queue and afterwards send a notification.
template <typename T>
void MessageQueue<T>::send(T &&msg)
{
    // modify vector under the unique lock
    std::lock_guard<std::mutex> uLock(_mutex);

    // push message to queue and notify client
    std::cout << "   Message #" << msg << " is being sent to the queue" << std::endl;
    _queue.push_back(std::move(msg));
    _cond.notify_one();
}

/* Implementation of class "TrafficLight" */

TrafficLight::TrafficLight()
{
    _currentPhase = TrafficLightPhase::red;
}

// infinitely runs and repeatedly calls the receive function on the message queue.
// Once it receives TrafficLightPhase::green, the method returns.
void TrafficLight::waitForGreen()
{
    while (true)
    {
        // sleep at every iteration to reduce CPU overhead
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // wake when message is received
        TrafficLightPhase trafficColorFrmMsg = _messageQueue.receive();
        if (trafficColorFrmMsg == TrafficLightPhase::green)
        {
            return;
        }
    }
}

TrafficLightPhase TrafficLight::getCurrentPhase()
{
    return _currentPhase;
}

// use the thread queue in the base class to cycle through phases on a thread.
void TrafficLight::simulate()
{
    threads.emplace_back(std::thread(&TrafficLight::cycleThroughPhases, this));
}

// virtual function which is executed in a thread
// the function runs an infinite loop that measures the time between two loop cycles
// and toggles the current phase of the traffic light between red and green and sends an update method
// to the message queue using move semantics. The cycle duration should be a random value between 4 and 6 seconds.
void TrafficLight::cycleThroughPhases()
{
    // generate random number uniformly from a range
    std::random_device randDev;
    std::mt19937 eng(randDev());
    std::uniform_real_distribution<> unif_dist(4000.0, 6000.0);

    double cycleDuration = unif_dist(eng); // single cycle duration in ms in simulation
    std::chrono::time_point<std::chrono::system_clock> lastUpdate;

    // initialize stop watch
    lastUpdate = std::chrono::system_clock::now();

    while (true)
    {
        // sleep at every iteration to reduce CPU overhead
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        // compute time difference to stop watch
        long timeSinceLastUpdate = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - lastUpdate).count();

        // check if cycle duration reached, to toggle traffic phase
        if (timeSinceLastUpdate >= cycleDuration)
        {
            // toggle traffic phase
            if (_currentPhase == TrafficLightPhase::red)
            {
                _currentPhase = TrafficLightPhase::green;
            }
            else
            {
                _currentPhase = TrafficLightPhase::red;
            }

            // use move semantics to send update message to queue
            _messageQueue.send(std::move(_currentPhase));

            // reset for next cycle
            lastUpdate = std::chrono::system_clock::now();
        }
    }
}