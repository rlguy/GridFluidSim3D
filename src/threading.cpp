/*
Copyright (c) 2016 Ryan L. Guy
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
#include "threading.h"


std::vector<std::pair<int, int> > Threading::splitIntoIndexRanges(int numElements, int numRanges) {
    std::vector<std::pair<int, int> > indexranges;

    if (numElements < numRanges) {
        for (int i = 0; i < numElements; i++) {
            indexranges.push_back(std::pair<int, int>(i, i));
        }

        return indexranges;
    }

    int chunksize = numElements / numRanges;
    for (int i = 0; i < numRanges; i++) {
        int start = (i == 0) ? 0 : indexranges[i - 1].second + 1;
        int end = (i == numRanges - 1) ? numElements - 1 : start + chunksize - 1;
        indexranges.push_back(std::pair<int, int>(start, end));
    }

    return indexranges;
}


std::vector<Threading::IndexRangeThreadParams> Threading::getIndexRangeThreadParams(
                                                            std::vector<std::pair<int, int> >ranges, 
                                                            void *obj) {
    std::vector<IndexRangeThreadParams> params(ranges.size());
    for (unsigned int i = 0; i < ranges.size(); i++) {
        params[i].startIndex = ranges[i].first;
        params[i].endIndex = ranges[i].second;
        params[i].obj = obj;
    }

    return params;
}

pthread_attr_t Threading::createJoinableThreadAttribute() {
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    return attr;
}

void Threading::destroyThreadAttribute(pthread_attr_t *attr) {
    pthread_attr_destroy(attr);
}

bool Threading::createThread(pthread_t *thread, const pthread_attr_t *attr, 
                             void *(*routine) (void *), void *arg) {
    bool failed = pthread_create(thread, attr, routine, arg);
    return !failed;
}

bool Threading::joinThreads(std::vector<pthread_t> &threads) {
    bool success = true;
    void *status;
    for(unsigned int i = 0; i < threads.size(); i++) {
        int result = pthread_join(threads[i], &status);
        if (result) {
            success = false;
        }
    }

    return success;
}

bool Threading::joinThread(pthread_t thread) {
    int result = pthread_join(thread, nullptr);
    if (result) {
        return false;
    }

    return true;
}

void Threading::splitIndexRangeWorkIntoThreads(int numElements, int numThreads, 
                                               void *obj, void *(*routine) (void *)) {
    std::vector<std::pair<int, int> > indexranges = Threading::splitIntoIndexRanges(
                                                                    numElements, numThreads);

    numThreads = indexranges.size();
    std::vector<Threading::IndexRangeThreadParams> params = Threading::getIndexRangeThreadParams(indexranges, obj);
    std::vector<pthread_t> threads(numThreads);

    pthread_attr_t attr = Threading::createJoinableThreadAttribute();
    for (int i = 0; i < numThreads; i++) {
        bool success = Threading::createThread(&threads[i], &attr, routine, (void *) &params[i]);
        assert(success);
    }
    Threading::destroyThreadAttribute(&attr);

    bool success = Threading::joinThreads(threads);
    assert(success);
}