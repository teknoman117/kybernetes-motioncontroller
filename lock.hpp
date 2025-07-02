#ifndef LOCK_HPP
#define LOCK_HPP

template<typename Sem>
class Lock final {
    Sem *sem;

public:
    Lock() = delete;
    Lock(const Lock&) = delete;
    Lock& operator=(const Lock&) = delete;

    Lock(Sem *sem_)
            : sem(sem_) {
        chSemWait(sem);
    }

    ~Lock() {
        if (sem) {
            chSemSignal(sem);
        }
    }

    Lock(Lock&& lock_)
            : sem(nullptr) {
        sem = lock_.sem;
        lock_.sem = nullptr;
    }

    Lock& operator=(Lock&& lock_) {
        sem = lock_.sem;
        lock_.sem = nullptr;
        return *this;
    }
};

#endif /* LOCK_HPP */