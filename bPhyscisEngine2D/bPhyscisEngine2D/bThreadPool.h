#pragma once
#include <exception>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>
#include <vector>
#include <queue>
#include <deque>

class bThreadPool {
public:
	bThreadPool(size_t);
	template<class F, class... Args>
	auto enqueue(F&& f, Args&&... args)
		->std::future<typename std::result_of<F(Args...)>::type>;
	~bThreadPool();
	void myWait();
private:
	// need to keep track of threads so we can join them
	std::vector< std::thread > workers;
	// the task queue
	std::queue< std::function<void()> > tasks;

	// synchronization
	std::mutex queue_mutex;
	std::condition_variable condition;
	bool stop;
};

// the constructor just launches some amount of workers
inline bThreadPool::bThreadPool(size_t threads)
	: stop(false)
{
	try {
		for (size_t i = 0; i<threads; ++i)
			workers.emplace_back(
					[this]
			{
				for (;;)
				{
					std::function<void()> task;

					{
						std::unique_lock<std::mutex> lock(this->queue_mutex);
						this->condition.wait(lock,
							[this] { return this->stop || !this->tasks.empty(); });
						if (this->stop && this->tasks.empty())
							return;
						task = std::move(this->tasks.front());
						this->tasks.pop();
					}

					task();
				}
			}
		);
	}
	catch (const std::exception& e)
	{
		std::cout << "Exception in workers.emplace_back: " << e.what() << std::endl;
	}
}

// add new work item to the pool
template<class F, class... Args>
auto bThreadPool::enqueue(F&& f, Args&&... args)
-> std::future<typename std::result_of<F(Args...)>::type>
{
	using return_type = typename std::result_of<F(Args...)>::type;

	auto task = std::make_shared< std::packaged_task<return_type()> >(
		std::bind(std::forward<F>(f), std::forward<Args>(args)...)
		);

	std::future<return_type> res = task->get_future();
	{
		std::unique_lock<std::mutex> lock(queue_mutex);

		// don't allow enqueueing after stopping the pool
		if (stop)
			throw std::runtime_error("enqueue on stopped bThreadPool");

		try
		{
			tasks.emplace([task]() { (*task)(); });
		}
		catch (const std::exception& e)
		{
			std::cout << "Exception in bThread: " << e.what() << std::endl;
		}
		
	}
	condition.notify_one();
	return res;
}

// the destructor joins all threads
inline bThreadPool::~bThreadPool()
{
	{
		std::unique_lock<std::mutex> lock(queue_mutex);
		stop = true;
	}
	condition.notify_all();
	for (std::thread &worker : workers)
	{
		try
		{
			worker.join();
		}
		catch (const std::exception& e)
		{
			std::cout << "Exception in ~bThreadPool: " << e.what() << std::endl;
		}
	}
		
}

inline void bThreadPool::myWait()
{
	bool isNotReady = true;

	while (isNotReady)
	{
		if (tasks.empty())
		{
			isNotReady = false;
		}
	}

}