#pragma once

namespace Auto
{
    struct Action
    {
        public:
            Action(void (*action), bool (*shouldStart), bool (*shouldStop));

            void (*action);
            bool (*shouldStart);
            bool (*shouldStop);
    };
}