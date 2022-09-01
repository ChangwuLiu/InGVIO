#pragma once

#include <vector>
#include <ctime>
#include <algorithm>
#include <cmath>

namespace feature_tracker
{
    class RandomPairGenerator
    {
    public:
        RandomPairGenerator(unsigned int n = 50) : xa(static_cast<int>(n*(n-1)/2)), count(0), N(n) 
        {
            for (int i = 0; i < static_cast<int>(n*(n-1)/2); ++i)
                xa[i] = i;
            std::random_shuffle(xa.begin(), xa.end());
        }
        RandomPairGenerator(RandomPairGenerator& obj) = delete;
        ~RandomPairGenerator(){}
        
        static void setRandomPairSeed()
        {
            srand((unsigned)time(NULL));
        }
        
        void generatePair(std::vector<unsigned int>& pair)
        {
            pair.clear();
            unsigned int i = xa[count];
            
            float k_max = (1.0+sqrt(i*8+9))/2.0;
            float k_min = (-1.0+sqrt(i*8+9))/2.0;
            
            float k_guess = floor(k_max);
            unsigned int k;
            
            if (k_guess < k_max && k_guess >= k_min)
                k = static_cast<unsigned int>(k_guess);
            else
                k = static_cast<unsigned int>(k_guess)-1;
            
            pair.push_back(N-k-1);
            pair.push_back(N-k+i-static_cast<unsigned int>(k*(k-1)/2));
            
            ++count;
            if (count >= static_cast<int>(N*(N-1)/2)) count = 0;
        }
    private:
        std::vector<unsigned int> xa;
        unsigned int count;
        unsigned int N;
    };
}

