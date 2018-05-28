# analysis.py
# -----------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


######################
# ANALYSIS QUESTIONS #
######################

# Set the given parameters to obtain the specified policies through
# value iteration.

def question2():
    answerDiscount = 0.9
    answerNoise = 0.2
    return answerDiscount, answerNoise

def question3a():
    answerDiscount = 0.5 # Low, prefer the closer terminal state
    answerNoise = 0.05 # Low, the cliff can be risked
    answerLivingReward = -0.95 # Negative, living is expensive
    return answerDiscount, answerNoise, answerLivingReward

def question3b():
    answerDiscount = 0.5 # Low, prefer the closer terminal state
    answerNoise = 0.3 # High, avoid the cliff
    answerLivingReward = -0.95 # Negative, living is expensive
    return answerDiscount, answerNoise, answerLivingReward

def question3c():
    answerDiscount = 0.9 # High, allow the farther terminal state
    answerNoise = 0.05 # Low, the cliff can be risked
    answerLivingReward = 0.05 # Low, terminating is more valuable than living
    return answerDiscount, answerNoise, answerLivingReward

def question3d():
    answerDiscount = 0.95 # High, allow the farther terminal state
    answerNoise = 0.5 # High, avoid the cliff
    answerLivingReward = 0.05 # Low, terminating is more valuable than living
    return answerDiscount, answerNoise, answerLivingReward

def question3e():
    answerDiscount = 0.05 # Very low, terminal states become worthless
    answerNoise = 0.5 # High, avoid the cliff
    answerLivingReward = 0.95 # High, living is more valuable than terminating
    return answerDiscount, answerNoise, answerLivingReward

def question6():
    answerEpsilon = None
    answerLearningRate = None
    return answerEpsilon, answerLearningRate
    # If not possible, return 'NOT POSSIBLE'

if __name__ == '__main__':
    print('Answers to analysis questions:')
    import analysis
    for q in [q for q in dir(analysis) if q.startswith('question')]:
        response = getattr(analysis, q)()
        print('  Question %s:\t%s' % (q, str(response)))
