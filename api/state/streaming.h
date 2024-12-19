
////
// INTERNAL HEADER - DO NOT INCLUDE!!!!!


#ifndef __STREAMING_H__
#define __STREAMING_H__


//////
//
// Includes
//

// C++ STL
#include <queue>
#include <mutex>
#include <memory>
#include <condition_variable>

// Local includes
#include <api/state/../../on_tube_vis.h>



//////
//
// Classes
//

struct command
{
	bool result, not_handled = true;
	std::mutex mtx;
	std::condition_variable cv;

	virtual ~command() {}

	virtual bool handle (void) = 0;
	virtual const std::string& describe (void) = 0;

	bool notify_result (bool result)
	{
		std::lock_guard g(mtx);
		this->result = result;
		not_handled = false;
		cv.notify_all();
		return result;
	}

	bool fetch_result (void)
	{
		std::unique_lock l(mtx);
		while (not_handled)
			cv.wait(l);
		return result;
	}
};

class command_stream
{
	static std::mutex mtx;
	static std::queue<std::shared_ptr<command>> queue;
	static std::condition_variable cv;

public:
	static void push (const std::shared_ptr<command> &cmd) {
		assert(otv_instance && "INTERNAL LOGIC ERROR: command pushed before otv_instance is ready!");
		std::lock_guard g(mtx);
		queue.push(cmd);
		otv_instance->post_redraw();
		cv.notify_all();
	}

	static std::shared_ptr<command> fetch (void)
	{
		std::unique_lock l(mtx);
		while (queue.empty())
			cv.wait(l);
		auto cmd = std::move(queue.front());
		queue.pop();
		return cmd;
	}

	static std::shared_ptr<command> poll (void)
	{
		std::unique_lock l(mtx);
		if (queue.empty())
			return nullptr;
		auto cmd = std::move(queue.front());
		queue.pop();
		return cmd;
	}

	static bool has_pending (void) {
		std::unique_lock l(mtx);
		return !queue.empty();
	}
};


#endif // ifdef __STREAMING_H__
