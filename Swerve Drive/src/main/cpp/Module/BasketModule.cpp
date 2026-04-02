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
                Move(Constants::Basket::UP_POSITION);
                break;
            }
            case State::Down:
            {
                Move(Constants::Basket::DOWN_POSITION);
                break;
            }
            
        }   
    }
    BasketModule::State BasketModule::GetState()
    {
        return currentState;
    }
    
    void BasketModule::Move(double pos)
    {
        left.Set(pos);
        right.Set(1 - (pos / 6.1f));
    }

}