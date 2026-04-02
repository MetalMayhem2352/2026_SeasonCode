#include "Modules/BasketModule.h"

namespace Modules
{
    BasketModule::BasketModule()
    {
       
    }

    BasketModule::~BasketModule()
    {
    }

    void BasketModule::UpdateState(State newState)
    {
        currentState = newState;
        switch (newState)
        {
            case State::Up:
            {
                
                break;
            }
            case State::Down:
            {
                
                break;
            }
            case State::Sweep:
            {
                break;
            }
            
        }   
    }
    BasketModule::State BasketModule::GetState()
    {
        return currentState;
    }

}