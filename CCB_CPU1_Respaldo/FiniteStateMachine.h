// Max number of Out Transitions (just to avoid using malloc)
#define MaxTrans 4
#define NumStates 8



// GENERIC CODE FOR ANY FINITE STATE MACHINE

struct State{

    // single state definition. Contains transition logic and output info

    uint16_t name;                  // State identifier

    uint16_t numOutTrans;           // Number of outgoing transitions

    // the following arrays have corresponding indices
    uint16_t inputMasks[MaxTrans];      // Input bit selection mask
    uint16_t negateInput[MaxTrans];     // {0, 1} depending on whether input bit must be 1 or 0 to trigger transition.
                                        // Must be consistent with inputMask and all other bits must be set to 0.
    uint16_t nextState[MaxTrans];       // Next State corresponding to the input

    uint16_t outputVector;              // binary output int

};

uint16_t computeNext(struct State *pStateVar, uint16_t inputVector){

    // default transition is to itself
    uint16_t nextState = pStateVar->name;

    // loop through out transitions in order of priority
    int i;
    for(i = 0; i < pStateVar->numOutTrans; i++){

        // bit-wise AND to mask relevant bits
        uint16_t maskedInput = inputVector & pStateVar->inputMasks[i];
        // bit-wise XOR to negate bits that must be set to 0
        maskedInput = (maskedInput ^ pStateVar->negateInput[i]);

        if( maskedInput == pStateVar->inputMasks[i] ){
            nextState = pStateVar->nextState[i];
            // next transition is the first in priority list
            break;
        }
    }
    return nextState;
}

void StatePresentation(struct State *pStateVar){

    // only for debugging

    printf("my state number is: %d\n", pStateVar->name);
    printf("my out vector is: %d\n", pStateVar->outputVector);
    int t;
    for(t = 0; t < pStateVar->numOutTrans; t++){

        uint16_t inMask = pStateVar->inputMasks[t];
        uint16_t negated = pStateVar->negateInput[t];

        int bit;
        printf("if ");
        for (bit = 0; bit < 16; bit++){
            if (inMask & (1 << bit)){
                printf("input %d", bit);
                bool opposite = (bool) (negated & (1 << bit));
                printf(" is %d", (1- (uint8_t)opposite));
                printf(" and, ");
            }
        }

        printf("\nthen next state is: %d \n", pStateVar->nextState[t]);

    }
}


////////////////////////////////////////////////////////////////////////////////////////////////


struct FSM{

    // Moore Finite State Machine

    struct State stateSet[NumStates];  // Finite set of State structure pointers
    uint16_t initialState;             // start state which is an element of stateSet
    uint16_t currentState;             // Only non-constant member. Tracks the present state of the FSM

    uint16_t currentOutput;
    uint16_t initialOutput;
};

uint16_t stepFSM(struct FSM *pFSMvar, uint16_t inputVector, uint16_t *pOutputVector){

    uint16_t current = pFSMvar->currentState;
    uint16_t nextState = computeNext(&(pFSMvar->stateSet[current]), inputVector);

    pFSMvar->currentState = nextState;
    pFSMvar->currentOutput = pFSMvar->stateSet[nextState].outputVector;

    *pOutputVector = pFSMvar->currentOutput ;

    return pFSMvar->currentState;

}

void resetFSM(struct FSM *pFSMvar){

    pFSMvar->currentState = pFSMvar->initialState;
    pFSMvar->currentOutput = pFSMvar->initialOutput;

    return;

}

void FSMpresentation(struct FSM *pFSMvar){

    // only for debugging
    printf("\nThis FSM has states: \n");
    int i;
    for(i = 0; i< NumStates; i++){
        printf("\n");
        StatePresentation(&pFSMvar->stateSet[i]);
    }

}

////////////////////////////////////////////////////////////////////////////////////////////////

void clearVector(uint16_t *pVect){
    *pVect = 0;
}














