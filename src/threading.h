/*
Copyright (c) 2015 Ryan L. Guy
This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#ifndef THREADING_H
#define THREADING_H

#include <pthread.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <assert.h>

namespace Threading {

    class Mutex {
        public:
            Mutex() { pthread_mutex_init(&_mutex, nullptr); }
            ~Mutex() { pthread_mutex_destroy(&_mutex); }
            void lock() { pthread_mutex_lock(&_mutex); }
            void unlock() { pthread_mutex_unlock(&_mutex); }
            pthread_mutex_t *getMutex() { return &_mutex; }

            Mutex(const Mutex &obj) {
                pthread_mutex_init(&_mutex, nullptr);
            }

            Mutex operator=(const Mutex &rhs) {
                pthread_mutex_destroy(&_mutex);
                pthread_mutex_init(&_mutex, nullptr);

                return *this;
            }

        private:
            pthread_mutex_t _mutex;
    };

    class ConditionVariable {
        public:
            ConditionVariable() { pthread_cond_init(&_cond, nullptr); }
            ~ConditionVariable() { pthread_cond_destroy(&_cond); }
            void wait(Mutex *mutex) { pthread_cond_wait(&_cond, mutex->getMutex()); }
            void signal() { pthread_cond_signal(&_cond); }

            ConditionVariable(const ConditionVariable &obj) {
                pthread_cond_init(&_cond, nullptr);
            }

            ConditionVariable operator=(const ConditionVariable &rhs) {
                pthread_cond_destroy(&_cond);
                pthread_cond_init(&_cond, nullptr);

                return *this;
            }

        private:
            pthread_cond_t _cond;
    };
    
    struct IndexRangeThreadParams {
        int startIndex;
        int endIndex;
        void *obj;
    };

    extern std::vector<std::pair<int, int> > splitIntoIndexRanges(int numElements, int numRanges);
    extern std::vector<IndexRangeThreadParams> getIndexRangeThreadParams(
                                                    std::vector<std::pair<int, int> >ranges, void *obj);
    extern pthread_attr_t createJoinableThreadAttribute();
    extern void destroyThreadAttribute(pthread_attr_t *attr);
    extern bool createThread(pthread_t *thread, const pthread_attr_t *attr, 
                             void *(*routine) (void *), void *arg);
    extern bool joinThreads(std::vector<pthread_t> &threads);
    extern bool joinThread(pthread_t thread);
    extern void splitIndexRangeWorkIntoThreads(int numElements, int numThreads, 
                                               void *obj, void *(*routine) (void *));
}

#endif