#pragma once 
#include <functional>
#include <queue>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>


class Thread_Pool{

std::atomic_bool done;
std::queue<std::function<void()>> m_tasks;
std::vector<std::thread> threads; 
uint32_t thread_count = 0;




// Dedclartion
public: 
Thread_Pool(uint32_t num_threads){
    thread_count = num_threads;
}


};