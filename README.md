# Simple-Character-Animation-Reinforce-Learninng

Goal : 타격 강도와 목표 위치에 따른 팔의 움직임을 강화학습으로 생성


## Intro

OpenGL + Bullet Physics + Bullet Physics ExampleBrowser + [NN](https://github.com/jmhong-simulation/2016FallCSE2022/tree/master/Week15%20-%20RL/NeuralNetwork)

위와 같은 C++ 환경에서 강화학습 예제를 만들고 실험할 수 있는 개발 환경을 구현했다.


## Demo

### Sample Images

![img1](/_imgs/1.PNG)
![img2](/_imgs/2.PNG)


### YouTube

[Single Joint Example](https://youtu.be/-Ku9dZcwSQc)

[Two Joint Example](https://youtu.be/97vfoNJHUys)

[Linear Regression](https://youtu.be/8Jc_rD9IS-g)


## Architecture

ToDo:


## Libs

[Bullet Physics Library 2.85.1](https://github.com/bulletphysics/bullet3/tree/2.85.1)

[Bullet Physics ExampleBrowser](https://github.com/bulletphysics/bullet3/tree/2.85.1/examples/ExampleBrowser)

[Neural Network](https://github.com/jmhong-simulation/2016FallCSE2022/tree/master/Week15%20-%20RL/NeuralNetwork)


## Build

Visual Studio 14 2015 Win64.

run App.sln.


## Examples


### Shortcuts

- ESC : On/Off Rendering 
- Home : Init Enviroment
- End : On/Off Tranining
- Insert : On/Off Command line log
- Arrow Up : Up Shoulder Joint
- Arrow Down : Down Shoulder Joint
- Arrow Left : Up(In) Elbow Joint
- Arrow Right : Down(Out) Elbow Joint


### Add Examples

1. copy xxx exampel to yyy example
2. change filename yyy_env.cpp, yyy_env.h
3. change class name to yyy_env.cpp, yyy_env.h
4. add new example ExampleImporter.cpp


### List

- 000 : Simple Skeleton with Collision check
- 001 : Single Joint + Linear Regression
- 002 : Two Joint, Display Fist to Target(F2T) angle/distance, Shoulder angle, Elbow angle
- 003 : Fiie I/O, Memory Replay
- 010 : Two Joint + Linear Regression
- 011 : Two Joint + Q Learning, F2T distance Reward
- 012 : Two Joint + Q Learning, F2T distance/angle Reward
- 020 : lab0. (Two Joint Single Target + Q Learning + MemoryReplay + File I/O) Skeleton
- 021 : lab0. add angle
- 030 : Two Joint + Two Network
- 040 : Test Init Motion
- 050 : lab1. Enviroment Test
- 051 : lab1. using F2T distance
- 052 : lab1. using F2T distance/angle
- 053 : lab1. using All State
- 200 : labF. Single Joint (Shoulder) Sekelton using Angluar Velocitiy
- 201 : labF. Single Joint (Elbow) Sekelton using Angluar Velocitiy
- 202 : labF. Two Joint Sekelton using Angluar Velocitiy
- 203 : labF. Two Joint with Fixed starting position Sekelton using Angluar Velocitiy
- 210 : labF. Single Joint (Shoulder) + Random Target + Random Move
- 211 : labF. Single Joint (Shoulder) + Random Target + Normal RL
- 212 : labF. Single Joint (Shoulder) + Random Target + Normal RL(2)
- 213 : labF. Single Joint (Shoulder) + Random Target + RL using Experience Replay


## history

- [0.4.0](https://github.com/hyunjun529/Simple-Character-Animation-Reinforce-Learninng/releases/tag/0.4.0)
- [0.3.0](https://github.com/hyunjun529/Simple-Character-Animation-Reinforce-Learninng/releases/tag/0.3.0)
- [0.2.1](https://github.com/hyunjun529/Simple-Character-Animation-Reinforce-Learninng/releases/tag/0.2.1)
- [0.2.0](https://github.com/hyunjun529/Simple-Character-Animation-Reinforce-Learninng/releases/tag/0.2.0)
- [0.1.0](https://github.com/hyunjun529/Simple-Character-Animation-Reinforce-Learninng/releases/tag/0.1.0)
