#ifndef __GGP_ROBOT_KEY_LISTENER_H
#define __GGP_ROBOT_KEY_LISTENER_H

#include <termios.h>
#include <stdio.h>
#include <boost/thread.hpp>

// listens asynchronously for pressed keys (characters) and stores the most
// recent one
class KeyListener {

  public:
    KeyListener();
    ~KeyListener();
    void start();
    char getChar();

  private:
    // here the last char is stored. make sure to lock on mutex when altering
    char c;
    boost::mutex mtx;
    struct termios oldt;
    struct termios newt;
    boost::thread t;
    
    int getch(void);
    void doWork();
};

// init with null-char
inline KeyListener::KeyListener()
: c(0)
{
  // save current terminal options
  tcgetattr(STDIN_FILENO, &oldt);

  // create new terminal options:
  // - non-canonical: read single char only
  // - non-echo: do not echo the typed char
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
}

// if destructed: reset terminal options
inline KeyListener::~KeyListener() {
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

// start a new thread and execute the doWork function
inline void KeyListener::start() {
  t = boost::thread(boost::bind(&KeyListener::doWork, this));
}

// get the stored char thread-safe
inline char KeyListener::getChar() {
  mtx.lock();
  char copy = c;
  // reset the store to the null-char
  c = (char)0;
  mtx.unlock();
  return copy;
}


// activate special terminal options, read char and reset options
inline int KeyListener::getch() {
  int ch;
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}

// thread-function, looping and waiting for pressed keys
inline void KeyListener::doWork (){
  while(1) {
    char copy;
    copy = getch();
    // store char
    mtx.lock();
    c = copy;
    mtx.unlock();
  }
} 

#endif
