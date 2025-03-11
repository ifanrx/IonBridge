// singleton.h
#ifndef SINGLETON_H
#define SINGLETON_H

template <typename T>
class Singleton {
 public:
  // Deleted copy constructor and assignment operator to prevent copies
  Singleton(const Singleton&) = delete;
  Singleton& operator=(const Singleton&) = delete;

  // Provides access to the Singleton instance
  static T& GetInstance() {
    // Guaranteed to be thread-safe in C++11 and later
    static T instance;
    return instance;
  }

 protected:
  // Protected constructor and destructor to allow inheritance
  Singleton() = default;
  virtual ~Singleton() = default;
};

#endif  // SINGLETON_H
