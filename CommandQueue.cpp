/*********************************************************************************
*  Copyright (c) 2010-2011, Elliott Cooper-Balis
*                             Paul Rosenfeld
*                             Bruce Jacob
*                             University of Maryland 
*                             dramninjas [at] gmail [dot] com
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/








//CommandQueue.cpp
//
//Class file for command queue object
//

#include "CommandQueue.h"
#include "MemoryController.h"
#include <assert.h>
#include "Rank.h"

using namespace DRAMSim;

CommandQueue::CommandQueue(vector< vector<BankState> > &states, ostream &dramsim_log_) :
		dramsim_log(dramsim_log_),
		bankStates(states),
		nextBank(0),
		nextRank(0),
		nextBankPRE(0),
		nextRankPRE(0),
		refreshRank(0),
		refreshWaiting(false),
		sendAct(true)
{
	//set here to avoid compile errors
	currentClockCycle = 0;

	//use numBankQueus below to create queue structure
	size_t numBankQueues;
	if (queuingStructure==PerRank)
	{
		numBankQueues = 1;
	}
	else if (queuingStructure==PerRankPerBank)
	{
		numBankQueues = NUM_BANKS;
	}
	else
	{
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}

	//vector of counters used to ensure rows don't stay open too long
	rowAccessCounters = vector< vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS,0));
    
    //vectors of counters used for hitrate and access rate of a specific bank
    bankHitCounters = vector< vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS,0));
    bankAccessCounters = vector< vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS,0));
    bankRowBufferPolicy = vector< vector<RowBufferPolicy> >(NUM_RANKS, vector<RowBufferPolicy>(NUM_BANKS,rowBufferPolicy));
    
    rowIdleProblemForClosePagePolicy = vector< vector<bool> >(NUM_RANKS, vector<bool>(NUM_BANKS,false));
    rowIdleProblemForOpenPagePolicy = vector< vector<bool> >(NUM_RANKS, vector<bool>(NUM_BANKS,false));

    rowActiveProblemForClosePagePolicy = vector< vector<bool> >(NUM_RANKS, vector<bool>(NUM_BANKS,false));
    readWriteRowActiveProblemForClosePagePolicy = vector< vector<bool> >(NUM_RANKS, vector<bool>(NUM_BANKS,false));
    
	//create queue based on the structure we want
	BusPacket1D actualQueue;
	BusPacket2D perBankQueue = BusPacket2D();
	queues = BusPacket3D();
	for (size_t rank=0; rank<NUM_RANKS; rank++)
	{
		//this loop will run only once for per-rank and NUM_BANKS times for per-rank-per-bank
		for (size_t bank=0; bank<numBankQueues; bank++)
		{
			actualQueue	= BusPacket1D();
			perBankQueue.push_back(actualQueue);
		}
		queues.push_back(perBankQueue);
	}


	//FOUR-bank activation window
	//	this will count the number of activations within a given window
	//	(decrementing counter)
	//
	//countdown vector will have decrementing counters starting at tFAW
	//  when the 0th element reaches 0, remove it
	tFAWCountdown.reserve(NUM_RANKS);
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		//init the empty vectors here so we don't seg fault later
		tFAWCountdown.push_back(vector<unsigned>());
	}
    previousPacket = NULL;
    //previousPacketsArray = vector< vector<BusPacket> >(NUM_RANKS, vector<BusPacket>(NUM_BANKS,*new BusPacket()));
    previousPacketsArray = vector< vector<BusPacket *> >(NUM_RANKS, vector<BusPacket *>(NUM_BANKS,NULL));
    
    switchedToClosePage = vector< vector<bool> > (NUM_RANKS, vector<bool> (NUM_BANKS, false));
    openPageRestoreDone = vector< vector<bool> > (NUM_RANKS, vector<bool> (NUM_BANKS, false));
    unifiedSaturatingCounter = 0;
    distributedSaturatingCounter = vector< vector<unsigned> >(NUM_RANKS, vector<unsigned>(NUM_BANKS,3));
    
    cycleOfRowConflict = vector< vector<uint64_t> > (NUM_RANKS, vector<uint64_t>(NUM_BANKS, 0));
    rowConflictHappened = vector< vector<bool> > (NUM_RANKS, vector<bool> (NUM_BANKS, false));

}
CommandQueue::~CommandQueue()
{
	//ERROR("COMMAND QUEUE destructor");
	size_t bankMax = NUM_RANKS;
	if (queuingStructure == PerRank) {
		bankMax = 1; 
	}
	for (size_t r=0; r< NUM_RANKS; r++)
	{
		for (size_t b=0; b<bankMax; b++) 
		{
			for (size_t i=0; i<queues[r][b].size(); i++)
			{
				delete(queues[r][b][i]);
			}
			queues[r][b].clear();
		}
	}
}
//Adds a precharge command when turning from the open page to close page
void CommandQueue::enqueuePrecharge(unsigned rank, unsigned bank)
{
    //unsigned rank = newBusPacket->rank;
    //unsigned bank = newBusPacket->bank;
    BusPacket *PrechargeBusPacket = new BusPacket(PRECHARGE, 0, 0, 0, rank, bank, 0, dramsim_log);
    vector<BusPacket *> &queue = getCommandQueue(rank, bank);
    queue.insert(queue.begin(), PrechargeBusPacket);
    

}

//Adds a command to appropriate queue
void CommandQueue::enqueue(BusPacket *newBusPacket)
{
	unsigned rank = newBusPacket->rank;
	unsigned bank = newBusPacket->bank;
	if (queuingStructure==PerRank)
	{
		queues[rank][0].push_back(newBusPacket);
		if (queues[rank][0].size()>CMD_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
	else if (queuingStructure==PerRankPerBank)
	{
		queues[rank][bank].push_back(newBusPacket);
		if (queues[rank][bank].size()>CMD_QUEUE_DEPTH)
		{
			ERROR("== Error - Enqueued more than allowed in command queue");
			ERROR("						Need to call .hasRoomFor(int numberToEnqueue, unsigned rank, unsigned bank) first");
			exit(0);
		}
	}
	else
	{
		ERROR("== Error - Unknown queuing structure");
		exit(0);
	}
}

//Removes the next item from the command queue based on the system's
//command scheduling policy
bool CommandQueue::pop(BusPacket **busPacket)
{
	//this can be done here because pop() is called every clock cycle by the parent MemoryController
	//	figures out the sliding window requirement for tFAW
	//
	//deal with tFAW book-keeping
	//	each rank has it's own counter since the restriction is on a device level
	for (size_t i=0;i<NUM_RANKS;i++)
	{
		//decrement all the counters we have going
		for (size_t j=0;j<tFAWCountdown[i].size();j++)
		{
			tFAWCountdown[i][j]--;
		}

		//the head will always be the smallest counter, so check if it has reached 0
		if (tFAWCountdown[i].size()>0 && tFAWCountdown[i][0]==0)
		{
			tFAWCountdown[i].erase(tFAWCountdown[i].begin());
		}
	}

	/* Now we need to find a packet to issue. When the code picks a packet, it will set
		 *busPacket = [some eligible packet]
		 
		 First the code looks if any refreshes need to go
		 Then it looks for data packets
		 Otherwise, it starts looking for rows to close (in open page)
	*/
    if (HYBRID_PAGE_POLICY_FLAG == true && DISTRIBUTED_PAGE_POLICY_FLAG == true)
    {
        unsigned startingRank = nextRank;
        unsigned startingBank = nextBank;
        bool foundIssuable = false;
        do
        {
            
            if (bankRowBufferPolicy[nextRank][nextBank]==ClosePage)
            {
                bool sendingREF = false;
                //if the memory controller set the flags signaling that we need to issue a refresh
                //if we're not sending a REF, proceed as normal
                if (!sendingREF)
                {
                    //bool foundIssuable = false;
                    //unsigned startingRank = nextRank;
                    //unsigned startingBank = nextBank;
                    
                    vector<BusPacket *> &queue = getCommandQueue(nextRank, nextBank);
                    //make sure there is something in this queue first
                    //	also make sure a rank isn't waiting for a refresh
                    //	if a rank is waiting for a refesh, don't issue anything to it until the
                    //		refresh logic above has sent one out (ie, letting banks close)
                    if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
                    {
                        if (queuingStructure == PerRank)
                        {
                            
                            //search from beginning to find first issuable bus packet
                            
                            for (size_t i=0;i<queue.size();i++)
                            {
                                if (isIssuable(queue[i]))
                                {
                                    
                                    //check to make sure we aren't removing a read/write that is paired with an activate
                                    if (i>0 && queue[i-1]->busPacketType==ACTIVATE &&
                                        queue[i-1]->physicalAddress == queue[i]->physicalAddress)
                                        continue;
                                        
                                    *busPacket = queue[i];
                                    //check to see if the packet has not been accessed before
                                    // and if so, we could count it as a hit
                                    hit(*busPacket);
                                    bankAccess(*busPacket);
                                    queue.erase(queue.begin()+i);
                                    foundIssuable = true;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            if (rowAccessCounters[queue[0]->rank][queue[0]->bank] == TOTAL_ROW_ACCESSES)
                            {
                                rowAccessCounters[queue[0]->rank][queue[0]->bank]--;
                            }
                            //PRINT("Checking that packet is issuable");
                            if (isIssuable(queue[0]))
                            {
                                
                                //no need to search because if the front can't be sent,
                                // then no chance something behind it can go instead
                                
                                *busPacket = queue[0];
                                
                                // set the restore part
                                //check to see if the packet has not been accessed before
                                // and if so, we could count it as a hit
                                hit(*busPacket);
                                bankAccess(*busPacket);
                                
                                queue.erase(queue.begin());
                                foundIssuable = true;
                            }
                            /*
                            else
                            {
                                if (switchedToClosePage[queue[0]->rank][queue[0]->bank] && bankStates[queue[0]->rank][queue[0]->bank].lastCommand != ACTIVATE)
                                //if (switchedToClosePage[queue[0]->rank][queue[0]->bank])
                                {
                                    BusPacket *PreCommand = new BusPacket(PRECHARGE, queue[0]->physicalAddress,
                                                                          queue[0]->column, bankStates[queue[0]->rank][queue[0]->bank].openRowAddress, queue[0]->rank,
                                                                          queue[0]->bank, 0, dramsim_log);
                                    if (isIssuable(PreCommand))
                                    {
                                        //PRINT("Activate packet is popped");
                                        *busPacket = PreCommand;
                                        switchedToClosePage[queue[0]->rank][queue[0]->bank] = false;
                                        foundIssuable = true;
                                    }
                                }
                                //if (rowIdleForClosePagePolicy)
                                else if (rowIdleProblemForClosePagePolicy[queue[0]->rank][queue[0]->bank] == true)
                                {
                                    //PRINT("row idle problem");
                                    BusPacket *ACTcommand = new BusPacket(ACTIVATE, queue[0]->physicalAddress,
                                                                          queue[0]->column, queue[0]->row, queue[0]->rank,
                                                                          queue[0]->bank, 0, dramsim_log);
                                    if (isIssuable(ACTcommand))
                                    {
                                        //PRINT("Activate packet is popped");
                                        *busPacket = ACTcommand;
                                        //rowIdleForClosePagePolicy = false;
                                        rowIdleProblemForClosePagePolicy[queue[0]->rank][queue[0]->bank] = false;
                                        foundIssuable = true;
                                    }
                                }
                                
                                //else if (rowActiveForClosePagePolicy)
                                else if (rowActiveProblemForClosePagePolicy[queue[0]->rank][queue[0]->bank] == true)
                                {
                                    //PRINT("bank precharged");
                                    BusPacket *PreCommand = new BusPacket(PRECHARGE, queue[0]->physicalAddress,
                                                                          queue[0]->column, bankStates[queue[0]->rank][queue[0]->bank].openRowAddress, queue[0]->rank,
                                                                          queue[0]->bank, 0, dramsim_log);
                                    if (isIssuable(PreCommand))
                                    {
                                        *busPacket = PreCommand;
                                        //rowActiveForClosePagePolicy = false;
                                        rowActiveProblemForClosePagePolicy[queue[0]->rank][queue[0]->bank] = false;
                                        foundIssuable = true;
                                    }
                                    
                                }
                                
                                else if (readWriteRowActiveProblemForClosePagePolicy[queue[0]->rank][queue[0]->bank] == true)
                                {

                                    
                                    BusPacket *PreCommand = new BusPacket(PRECHARGE, queue[0]->physicalAddress,
                                                                          queue[0]->column, bankStates[queue[0]->rank][queue[0]->bank].openRowAddress, queue[0]->rank,
                                                                          queue[0]->bank, 0, dramsim_log);
                                    if (isIssuable(PreCommand))
                                    {
                                        *busPacket = PreCommand;
                                        //rowActiveForClosePagePolicy = false;
                                        readWriteRowActiveProblemForClosePagePolicy[queue[0]->rank][queue[0]->bank] = false;
                                        foundIssuable = true;
                                    }
                                    
                                }
                            }
                            */
                        }
                        
                        if (foundIssuable == false)
                        {
                            if (HYBRID_PAGE_POLICY_FLAG == true)
                            {
                                
                                //PRINT("not found");
                                if (ENABLE_RESTORE == true)
                                {
                                    //PRINT("not found");
                                    /*
                                    PRINT("last command: " << bankStates[nextRank][nextBank].lastCommand);
                                    PRINT("current bank state: " << bankStates[nextRank][nextBank].currentBankState);
                                    PRINT("bus packet type " << queue[0]->busPacketType);
                                    PRINT("bus packet row " << queue[0]->row);
                                    PRINT("bank open row " << bankStates[nextRank][nextBank].openRowAddress);
                                    PRINT("total row access: " << rowAccessCounters[nextRank][nextBank]);
                                    */
                                    if ((bankStates[nextRank][nextBank].lastCommand == READ || bankStates[nextRank][nextBank].lastCommand == WRITE || bankStates[nextRank][nextBank].lastCommand == ACTIVATE) && bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row != bankStates[nextRank][nextBank].openRowAddress)
                                    {
                                        if (queue[0]->busPacketType == ACTIVATE)
                                        {
                                            
                                            if (currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                            {
                                                if (openPageRestoreDone[nextRank][nextBank] == false)
                                                {
                                                    if (currentClockCycle >= bankStates[nextRank][nextBank].nextWrite)
                                                    {
                                                        *busPacket = new BusPacket(WRITE_RESTORE_PAGE, 0, 0, bankStates[nextRank][nextBank].openRowAddress, nextRank, nextBank, 0, dramsim_log);
                                                        openPageRestoreDone[nextRank][nextBank] = true;
                                                        foundIssuable = true;
                                                        //break;
                                                    }
                                                }
                                                //else if (currentClockCycle >= bankStates[nextRank][nextBank].nextPrecharge && bankStates[nextRank][nextBank].lastCommand != ACTIVATE)
                                                else if (currentClockCycle >= bankStates[nextRank][nextBank].nextPrecharge)
                                                    
                                                {
                                                    //PRINT("precharge row");
                                                    *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRank, nextBank, 0, dramsim_log);
                                                    
                                                    foundIssuable = true;
                                                    
                                                    openPageRestoreDone[nextRank][nextBank] = false;
                                                    //break;
                                                }
                                            }
                                        }
                                    }
                                    else if (bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row == bankStates[nextRank][nextBank].openRowAddress && currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                    {
                                        queue.erase(queue.begin());
                                    }
                                    else if (bankStates[nextRank][nextBank].currentBankState == Idle && (queue[0]->busPacketType == READ || queue[0]->busPacketType == READ_P || queue[0]->busPacketType == WRITE || queue[0]->busPacketType == WRITE_P) && currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                    {
                                        *busPacket = new BusPacket(ACTIVATE, 0, 0, queue[0]->row, queue[0]->rank, queue[0]->bank, 0, dramsim_log);
                                        openPageRestoreDone[nextRank][nextBank] = true;
                                        foundIssuable = true;
                                    }
                                }
                                
                                else
                                {
                                    /*
                                     PRINT("last command: " << bankStates[nextRank][nextBank].lastCommand);
                                     PRINT("current bank state: " << bankStates[nextRank][nextBank].currentBankState);
                                     PRINT("bus packet type " << queue[0]->busPacketType);
                                     PRINT("bus packet row " << queue[0]->row);
                                     PRINT("bank open row " << bankStates[nextRank][nextBank].openRowAddress);
                                     PRINT("total row access: " << rowAccessCounters[nextRank][nextBank]);
                                     */
                                    if ((bankStates[nextRank][nextBank].lastCommand == READ || bankStates[nextRank][nextBank].lastCommand == WRITE) && bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row != bankStates[nextRank][nextBank].openRowAddress)
                                    {
                                        if (currentClockCycle >= bankStates[nextRank][nextBank].nextPrecharge)
                                            
                                        {
                                            //PRINT("precharge row");
                                            *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRank, nextBank, 0, dramsim_log);
                                            
                                            foundIssuable = true;
                                            
                                            openPageRestoreDone[nextRank][nextBank] = false;
                                            //break;
                                        }
                                    }
                                    else if (bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row == bankStates[nextRank][nextBank].openRowAddress && currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                    {
                                        queue.erase(queue.begin());
                                    }
                                }
                            }
                        }
                        
                    }
                    
                    //if we found something, break out of do-while
                    if (foundIssuable)
                    {
                        break;
                    }
                    
                    /*
                     //rank round robin
                    if (queuingStructure == PerRank)
                    {
                        nextRank = (nextRank + 1) % NUM_RANKS;
                        if (startingRank == nextRank)
                        {
                            break;
                        }
                    }
                    else
                    {
                        nextRankAndBank(nextRank, nextBank);
                        if (startingRank == nextRank && startingBank == nextBank)
                        {
                            break;
                        }
                    }
                    
                    */
                    //if (!foundIssuable) return false;
                    
                }
            }
            else if (bankRowBufferPolicy[nextRank][nextBank]==OpenPage)
            {
                bool sendingREForPRE = false;
                
                if (!sendingREForPRE)
                {
                    //unsigned startingRank = nextRank;
                    //unsigned startingBank = nextBank;
                    //bool foundIssuable = false;
                    vector<BusPacket *> &queue = getCommandQueue(nextRank,nextBank);
                    //make sure there is something there first
                    if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
                    {
                        //search from the beginning to find first issuable bus packet
                        for (size_t i=0;i<queue.size();i++)
                        {
                            BusPacket *packet = queue[i];
                            if (isIssuable(packet))
                            {
                                //check for dependencies
                                bool dependencyFound = false;
                                for (size_t j=0;j<i;j++)
                                {
                                    BusPacket *prevPacket = queue[j];
                                    if (prevPacket->busPacketType != ACTIVATE &&
                                        prevPacket->bank == packet->bank &&
                                        prevPacket->row == packet->row)
                                    {
                                        //PRINT("dependency found");
                                        dependencyFound = true;
                                        break;
                                    }
                                }
                                if (dependencyFound) continue;
                                
                                *busPacket = packet;
                                //PRINT("bank state " << bankStates[packet->rank][packet->bank].currentBankState);
                                //PRINT("issued packet bank = " << packet->bank);
                                //PRINT("issued packet type = " << packet->busPacketType);
                                //PRINT("");
                                hit(*busPacket);
                                bankAccess(*busPacket);
                                
                                // Oracle implementation
                                if (ORACLE == true)
                                {
                                    if (rowConflictHappened[(*busPacket)->rank][(*busPacket)->bank] == true)
                                    {
                                        cycleOfRowConflict[(*busPacket)->rank][(*busPacket)->bank] = currentClockCycle;
                                        rowConflictHappened[(*busPacket)->rank][(*busPacket)->bank] = false;
                                    }
                                }
                                //if the bus packet before is an activate, that is the act that was
                                //	paired with the column access we are removing, so we have to remove
                                //	that activate as well (check i>0 because if i==0 then theres nothing before it)
                                if (i>0 && queue[i-1]->busPacketType == ACTIVATE)
                                {
                                    rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
                                    // i is being returned, but i-1 is being thrown away, so must delete it here
                                    delete (queue[i-1]);
                                    
                                    // remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
                                    queue.erase(queue.begin()+i-1,queue.begin()+i+1);
                                }
                                else // there's no activate before this packet
                                {
                                    //or just remove the one bus packet
                                    queue.erase(queue.begin()+i);
                                }
                                foundIssuable = true;
                                break;
                            }
                        }
                    }
                    
                    //if we found something, break out of do-while
                    if (foundIssuable)
                    {
                        break;
                    }
                    
                    
                    
                    //if nothing was issuable, see if we can issue a PRE to an open bank
                    //	that has no other commands waiting
                    
                    if (!foundIssuable)
                    {
                        //search for banks to close
                        bool sendingPRE = false;
                        unsigned startingPRERank = nextRankPRE;
                        unsigned startingPREBank = nextBankPRE;
                        
                        do // round robin over all ranks and banks
                        {
                         
                            vector <BusPacket *> &queue = getCommandQueue(nextRankPRE, nextBankPRE);
                            bool found = false;
                            //check if bank is open
                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive && bankRowBufferPolicy[nextRankPRE][nextBankPRE] == OpenPage)
                            {
                                for (size_t i=0;i<queue.size();i++)
                                {
                                    //if there is something going to that bank and row, then we don't want to send a PRE
                                    /*
                                    if (queue[i]->bank == nextBankPRE &&
                                        queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
                                    {
                                        found = true;
                                        break;
                                    }
                                    */
                                    if (ENABLE_RESTORE)
                                    {
                                        for (size_t i=0;i<queue.size();i++)
                                        {
                                            //if there is something going to that bank and row, then we don't want to send a PRE
                                            if (queue[i]->bank == nextBankPRE &&
                                                queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
                                            {
                                                found = true;
                                                break;
                                            }
                                        }
                                    }
                                    else
                                    {
                                        for (size_t i=0;i<queue.size();i++)
                                        {
                                            //if there is something going to that bank and row, then we don't want to send a PRE
                                            if (queue[i]->bank == nextBankPRE &&
                                                queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress && bankStates[nextRankPRE][nextBankPRE].lastCommand == ACTIVATE)
                                            {
                                                found = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                                
                                //if nothing found going to that bank and row or too many accesses have happend, close it
                                if (!found || rowAccessCounters[nextRankPRE][nextBankPRE]==TOTAL_ROW_ACCESSES)
                                {
                                    /*
                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextPrecharge)
                                    {
                                        sendingPRE = true;
                                        *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                        //bankAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
                                        if (ENABLE_RESTORE)
                                        {
                                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
                                            {
                                                if (openPageRestoreDone[nextRankPRE][nextBankPRE] == false)
                                                {
                                                    
                                                    sendingPRE = false;
                                                    delete *busPacket;
                                                    
                                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextWrite)
                                                    {
                                                        *busPacket = new BusPacket(WRITE_RESTORE_PAGE, 0, 0, bankStates[nextRankPRE][nextBankPRE].openRowAddress, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                                        sendingPRE = true;
                                                        openPageRestoreDone[nextRankPRE][nextBankPRE] =true;
                                                        break;
                                                    }
                                                }
                                                else
                                                {
                                                    openPageRestoreDone[nextRankPRE][nextBankPRE] = false;
                                                    rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                                    break;
                                                }
                                            }
                                            else
                                            {
                                                PRINT("bank state : " << bankStates[nextRankPRE][nextBankPRE].currentBankState);
                                                break;
                                            }
                                        }
                                        rowAccessCounters[nextRankPRE][nextBankPRE] = 0;

                                        foundIssuable = true;
                                        break;
                                    }
                                    */
                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextPrecharge)
                                    {
                                        sendingPRE = true;
                                        foundIssuable = true;
                                        //rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                        *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                        if (ENABLE_RESTORE)
                                        {
                                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
                                            {
                                                
                                                if (openPageRestoreDone[nextRankPRE][nextBankPRE] == false)
                                                {
                                                    if (!found)//Oracle implementaion
                                                    {
                                                        rowConflictHappened[nextRankPRE][nextBankPRE] = true;
                                                    }
                                                    
                                                    sendingPRE = false;
                                                    delete *busPacket;
                                                    foundIssuable = false;
                                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextWrite)
                                                    {
                                                        *busPacket = new BusPacket(WRITE_RESTORE_PAGE, 0, 0, bankStates[nextRankPRE][nextBankPRE].openRowAddress, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                                        sendingPRE = true;
                                                        foundIssuable = true;
                                                        openPageRestoreDone[nextRankPRE][nextBankPRE] =true;
                                                        break;
                                                    }
                                                }
                                                else
                                                {
                                                    openPageRestoreDone[nextRankPRE][nextBankPRE] = false;
                                                    rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                                    break;
                                                }
                                            }
                                            else
                                            {
                                                PRINT("bank state : " << bankStates[nextRankPRE][nextBankPRE].currentBankState);
                                                break;
                                            }
                                        }
                                        else
                                        {
                                            if (!found)
                                            {
                                                rowConflictHappened[nextRankPRE][nextBankPRE] = true;
                                            }
                                        }
                                        
                                        rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                        break;
                                    }
                                }
                            }
                            
                            nextRankAndBank(nextRankPRE, nextBankPRE);
                        }
                        
                        while (!(startingPRERank == nextRankPRE && startingPREBank == nextBankPRE));
                        
                        
                        //if no PREs could be sent, just return false
                        //if (!sendingPRE) return false;
                    }
                }
            }
            
            if (foundIssuable)
            {
                break;
            }
            //rank round robin
            if (queuingStructure == PerRank)
            {
                nextRank = (nextRank + 1) % NUM_RANKS;
                if (startingRank == nextRank)
                {
                    break;
                }
            }
            else
            {
                nextRankAndBank(nextRank, nextBank);
                if (startingRank == nextRank && startingBank == nextBank)
                {
                    break;
                }
            }
            
        } while (true);
        
        //if we couldn't find anything to send, return false
        if (!foundIssuable) return false;
    }
    else
    {
        if (rowBufferPolicy==ClosePage)
        {
            bool sendingREF = false;
            //if the memory controller set the flags signaling that we need to issue a refresh
            if (refreshWaiting)
            {
                bool foundActiveOrTooEarly = false;
                //look for an open bank
                for (size_t b=0;b<NUM_BANKS;b++)
                {
                    vector<BusPacket *> &queue = getCommandQueue(refreshRank,b);
                    //checks to make sure that all banks are idle
                    if (bankStates[refreshRank][b].currentBankState == RowActive)
                    {
                        foundActiveOrTooEarly = true;
                        //if the bank is open, make sure there is nothing else
                        // going there before we close it
                        for (size_t j=0;j<queue.size();j++)
                        {
                            BusPacket *packet = queue[j];
                            if (packet->row == bankStates[refreshRank][b].openRowAddress &&
                                packet->bank == b)
                            {
                                if (packet->busPacketType != ACTIVATE && isIssuable(packet))
                                {
                                    *busPacket = packet;
                                    queue.erase(queue.begin() + j);
                                    sendingREF = true;
                                }
                                break;
                            }
                        }
                        
                        break;
                    }
                    //	NOTE: checks nextActivate time for each bank to make sure tRP is being
                    //				satisfied.	the next ACT and next REF can be issued at the same
                    //				point in the future, so just use nextActivate field instead of
                    //				creating a nextRefresh field
                    else if (bankStates[refreshRank][b].nextActivate > currentClockCycle)
                    {
                        foundActiveOrTooEarly = true;
                        break;
                    }
                }
                
                //if there are no open banks and timing has been met, send out the refresh
                //	reset flags and rank pointer
                if (!foundActiveOrTooEarly && bankStates[refreshRank][0].currentBankState != PowerDown)
                {
                    *busPacket = new BusPacket(REFRESH, 0, 0, 0, refreshRank, 0, 0, dramsim_log);
                    refreshRank = -1;
                    refreshWaiting = false;
                    sendingREF = true;
                }
            } // refreshWaiting
            
            //if we're not sending a REF, proceed as normal
            if (!sendingREF)
            {
                bool foundIssuable = false;
                unsigned startingRank = nextRank;
                unsigned startingBank = nextBank;
                do
                {
                    vector<BusPacket *> &queue = getCommandQueue(nextRank, nextBank);
                    //make sure there is something in this queue first
                    //	also make sure a rank isn't waiting for a refresh
                    //	if a rank is waiting for a refesh, don't issue anything to it until the
                    //		refresh logic above has sent one out (ie, letting banks close)
                    if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
                    {
                        if (queuingStructure == PerRank)
                        {
                            //search from beginning to find first issuable bus packet
                            for (size_t i=0;i<queue.size();i++)
                            {
                                if (isIssuable(queue[i]))
                                {
                                    //check to make sure we aren't removing a read/write that is paired with an activate
                                    if (i>0 && queue[i-1]->busPacketType==ACTIVATE &&
                                        queue[i-1]->physicalAddress == queue[i]->physicalAddress)
                                        continue;
                                    *busPacket = queue[i];
                                    queue.erase(queue.begin()+i);
                                    foundIssuable = true;
                                    hit(*busPacket);
                                    bankAccess(*busPacket);
                                    break;
                                }
                            }
                        }
                        else
                        {
                            if (HYBRID_PAGE_POLICY_FLAG)
                            {
                                if (rowAccessCounters[queue[0]->rank][queue[0]->bank] == TOTAL_ROW_ACCESSES)
                                {
                                    rowAccessCounters[queue[0]->rank][queue[0]->bank]--;
                                }
                            }
                            if (isIssuable(queue[0]))
                            {
                                //no need to search because if the front can't be sent,
                                // then no chance something behind it can go instead
                                *busPacket = queue[0];
                                queue.erase(queue.begin());
                                hit(*busPacket);
                                bankAccess(*busPacket);
                                foundIssuable = true;
                            }
                        }
                        
                        if (foundIssuable == false)
                        {
                            if (HYBRID_PAGE_POLICY_FLAG == true)
                            {
                                
                                //PRINT("not found");
                                if (ENABLE_RESTORE == true)
                                {
                                    //PRINT("not found");
                                    /*
                                     PRINT("last command: " << bankStates[nextRank][nextBank].lastCommand);
                                     PRINT("current bank state: " << bankStates[nextRank][nextBank].currentBankState);
                                     PRINT("bus packet type " << queue[0]->busPacketType);
                                     PRINT("bus packet row " << queue[0]->row);
                                     PRINT("bank open row " << bankStates[nextRank][nextBank].openRowAddress);
                                     PRINT("total row access: " << rowAccessCounters[nextRank][nextBank]);
                                     */
                                    if ((bankStates[nextRank][nextBank].lastCommand == READ || bankStates[nextRank][nextBank].lastCommand == WRITE || bankStates[nextRank][nextBank].lastCommand == ACTIVATE) && bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row != bankStates[nextRank][nextBank].openRowAddress)
                                    {
                                        if (queue[0]->busPacketType == ACTIVATE)
                                        {
                                            
                                            if (currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                            {
                                                if (openPageRestoreDone[nextRank][nextBank] == false)
                                                {
                                                    if (currentClockCycle >= bankStates[nextRank][nextBank].nextWrite)
                                                    {
                                                        *busPacket = new BusPacket(WRITE_RESTORE_PAGE, 0, 0, bankStates[nextRank][nextBank].openRowAddress, nextRank, nextBank, 0, dramsim_log);
                                                        openPageRestoreDone[nextRank][nextBank] = true;
                                                        foundIssuable = true;
                                                        //break;
                                                    }
                                                }
                                                //else if (currentClockCycle >= bankStates[nextRank][nextBank].nextPrecharge && bankStates[nextRank][nextBank].lastCommand != ACTIVATE)
                                                else if (currentClockCycle >= bankStates[nextRank][nextBank].nextPrecharge)
                                                    
                                                {
                                                    //PRINT("precharge row");
                                                    *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRank, nextBank, 0, dramsim_log);
                                                    
                                                    foundIssuable = true;
                                                    
                                                    openPageRestoreDone[nextRank][nextBank] = false;
                                                    //break;
                                                }
                                            }
                                        }
                                    }
                                    else if (bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row == bankStates[nextRank][nextBank].openRowAddress && currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                    {
                                        queue.erase(queue.begin());
                                    }
                                }
                                
                                else
                                {
                                    /*
                                     PRINT("last command: " << bankStates[nextRank][nextBank].lastCommand);
                                     PRINT("current bank state: " << bankStates[nextRank][nextBank].currentBankState);
                                     PRINT("bus packet type " << queue[0]->busPacketType);
                                     PRINT("bus packet row " << queue[0]->row);
                                     PRINT("bank open row " << bankStates[nextRank][nextBank].openRowAddress);
                                     PRINT("total row access: " << rowAccessCounters[nextRank][nextBank]);
                                     */
                                    if ((bankStates[nextRank][nextBank].lastCommand == READ || bankStates[nextRank][nextBank].lastCommand == WRITE) && bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row != bankStates[nextRank][nextBank].openRowAddress)
                                    {
                                        if (currentClockCycle >= bankStates[nextRank][nextBank].nextPrecharge)
                                            
                                        {
                                            //PRINT("precharge row");
                                            *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRank, nextBank, 0, dramsim_log);
                                            
                                            foundIssuable = true;
                                            
                                            openPageRestoreDone[nextRank][nextBank] = false;
                                            //break;
                                        }
                                    }
                                    else if (bankStates[nextRank][nextBank].currentBankState == RowActive && queue[0]->busPacketType == ACTIVATE && queue[0]->row == bankStates[nextRank][nextBank].openRowAddress && currentClockCycle >= bankStates[nextRank][nextBank].nextActivate)
                                    {
                                        queue.erase(queue.begin());
                                    }
                                }
                            }
                        }
                    }
                    
                    
                    //if we found something, break out of do-while
                    if (foundIssuable)
                    {
                        break;
                    }
                    
                    
                    //rank round robin
                    if (queuingStructure == PerRank)
                    {
                        nextRank = (nextRank + 1) % NUM_RANKS;
                        if (startingRank == nextRank)
                        {
                            break;
                        }
                    }
                    else
                    {
                        nextRankAndBank(nextRank, nextBank);
                        if (startingRank == nextRank && startingBank == nextBank)
                        {
                            break;
                        }
                    }
                }
                while (true);
                
                //if we couldn't find anything to send, return false
                if (!foundIssuable) return false;
            }
        }
        else if (rowBufferPolicy==OpenPage)
        {
            bool sendingREForPRE = false;
            if (refreshWaiting)
            {
                bool sendREF = true;
                //make sure all banks idle and timing met for a REF
                for (size_t b=0;b<NUM_BANKS;b++)
                {
                    //if a bank is active we can't send a REF yet
                    if (bankStates[refreshRank][b].currentBankState == RowActive)
                    {
                        sendREF = false;
                        bool closeRow = true;
                        //search for commands going to an open row
                        vector <BusPacket *> &refreshQueue = getCommandQueue(refreshRank,b);
                        
                        for (size_t j=0;j<refreshQueue.size();j++)
                        {
                            BusPacket *packet = refreshQueue[j];
                            //if a command in the queue is going to the same row . . .
                            if (bankStates[refreshRank][b].openRowAddress == packet->row &&
                                b == packet->bank)
                            {
                                // . . . and is not an activate . . .
                                if (packet->busPacketType != ACTIVATE)
                                {
                                    closeRow = false;
                                    // . . . and can be issued . . .
                                    if (isIssuable(packet))
                                    {
                                        //send it out
                                        *busPacket = packet;
                                        refreshQueue.erase(refreshQueue.begin()+j);
                                        sendingREForPRE = true;
                                    }
                                    break;
                                }
                                else //command is an activate
                                {
                                    //if we've encountered another act, no other command will be of interest
                                    break;
                                }
                            }
                        }
                        
                        //if the bank is open and we are allowed to close it, then send a PRE
                        if (closeRow && currentClockCycle >= bankStates[refreshRank][b].nextPrecharge)
                        {
                            rowAccessCounters[refreshRank][b]=0;
                            *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, refreshRank, b, 0, dramsim_log);
                            sendingREForPRE = true;
                        }
                        break;
                    }
                    //	NOTE: the next ACT and next REF can be issued at the same
                    //				point in the future, so just use nextActivate field instead of
                    //				creating a nextRefresh field
                    else if (bankStates[refreshRank][b].nextActivate > currentClockCycle) //and this bank doesn't have an open row
                    {
                        sendREF = false;
                        break;
                    }
                }
                
                //if there are no open banks and timing has been met, send out the refresh
                //	reset flags and rank pointer
                if (sendREF && bankStates[refreshRank][0].currentBankState != PowerDown)
                {
                    *busPacket = new BusPacket(REFRESH, 0, 0, 0, refreshRank, 0, 0, dramsim_log);
                    refreshRank = -1;
                    refreshWaiting = false;
                    sendingREForPRE = true;
                }
            }
            
            if (!sendingREForPRE)
            {
                unsigned startingRank = nextRank;
                unsigned startingBank = nextBank;
                bool foundIssuable = false;
                if (FIFO_OPEN_PAGE_SCHEDULING == true)
                {
                    
                    do // round robin over queues
                    {
                        vector<BusPacket *> &queue = getCommandQueue(nextRank,nextBank);
                        if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
                        {
                            if (queuingStructure == PerRank)
                            {
                                //make sure there is something there first
                                
                                //search from the beginning to find first issuable bus packet
                                for (size_t i=0;i<queue.size();i++)
                                {
                                    BusPacket *packet = queue[i];
                                    if (isIssuable(packet))
                                    {
                                        //check for dependencies
                                        bool dependencyFound = false;
                                        for (size_t j=0;j<i;j++)
                                        {
                                            BusPacket *prevPacket = queue[j];
                                            if (prevPacket->busPacketType != ACTIVATE &&
                                                prevPacket->bank == packet->bank &&
                                                prevPacket->row == packet->row)
                                            {
                                                dependencyFound = true;
                                                break;
                                            }
                                        }
                                        if (dependencyFound) continue;
                                        
                                        *busPacket = packet;
                                        
                                        //if the bus packet before is an activate, that is the act that was
                                        //	paired with the column access we are removing, so we have to remove
                                        //	that activate as well (check i>0 because if i==0 then theres nothing before it)
                                        if (i>0 && queue[i-1]->busPacketType == ACTIVATE)
                                        {
                                            rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
                                            // i is being returned, but i-1 is being thrown away, so must delete it here
                                            delete (queue[i-1]);
                                            
                                            // remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
                                            queue.erase(queue.begin()+i-1,queue.begin()+i+1);
                                        }
                                        else // there's no activate before this packet
                                        {
                                            //or just remove the one bus packet
                                            queue.erase(queue.begin()+i);
                                        }
                                        foundIssuable = true;
                                        break;
                                        
                                    }
                                }
                            }
                            
                            else
                            {
                                /*
                                if (rowAccessCounters[queue[0]->rank][queue[0]->bank] == TOTAL_ROW_ACCESSES)
                                {
                                    rowAccessCounters[queue[0]->rank][queue[0]->bank]--;
                                }
                                 */
                                if (isIssuable(queue[0]))
                                {
                                    //no need to search because if the front can't be sent,
                                    // then no chance something behind it can go instead
                                    *busPacket = queue[0];
                                    queue.erase(queue.begin());
                                    foundIssuable = true;
                                }
                                
                                
                            }
                        }
                        //if we found something, break out of do-while
                        if (foundIssuable)
                        {
                            break;
                        }
                        
                        //rank round robin
                        if (queuingStructure == PerRank)
                        {
                            nextRank = (nextRank + 1) % NUM_RANKS;
                            if (startingRank == nextRank)
                            {
                                break;
                            }
                        }
                        else
                        {
                            nextRankAndBank(nextRank, nextBank);
                            if (startingRank == nextRank && startingBank == nextBank)
                            {
                                break;
                            }
                        }
                    }
                    while (true);
                    
                    //if nothing was issuable, see if we can issue a PRE to an open bank
                    //	that has no other commands waiting
                    if (!foundIssuable)
                    {
                        //search for banks to close
                        bool sendingPRE = false;
                        unsigned startingRank = nextRankPRE;
                        unsigned startingBank = nextBankPRE;
                        
                        do // round robin over all ranks and banks
                        {
                            vector <BusPacket *> &queue = getCommandQueue(nextRankPRE, nextBankPRE);
                            bool found = false;
                            //check if bank is open
                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
                            {
                                if (queue[0]->restoreWrite == true)
                                {
                                    found = true;
                                }
                                /*
                                for (size_t i=0;i<queue.size();i++)
                                {
                                    //if there is something going to that bank and row, then we don't want to send a PRE
                                    
                                    if (queue[i]->bank == nextBankPRE &&
                                        queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
                                    {
                                        found = true;
                                        break;
                                    }
                                }
                                */
                                
                                
                                //if nothing found going to that bank and row or too many accesses have happend, close it
                                if (!found || rowAccessCounters[nextRankPRE][nextBankPRE]==TOTAL_ROW_ACCESSES)
                                {
                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextPrecharge && bankStates[nextRankPRE][nextBankPRE].lastCommand != ACTIVATE )
                                    {
                                        sendingPRE = true;
                                        
                                        //rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                        *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                        
                                        if (ENABLE_RESTORE)
                                        {
                                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
                                            {
                                                if (openPageRestoreDone[nextRankPRE][nextBankPRE] == false)
                                                {
                                                    
                                                    sendingPRE = false;
                                                    delete *busPacket;
                                                    
                                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextWrite)
                                                    {
                                                        *busPacket = new BusPacket(WRITE, 0, 0, bankStates[nextRankPRE][nextBankPRE].openRowAddress, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                                        sendingPRE = true;
                                                        openPageRestoreDone[nextRankPRE][nextBankPRE] =true;
                                                        break;
                                                    }
                                                }
                                                else
                                                {
                                                    openPageRestoreDone[nextRankPRE][nextBankPRE] = false;
                                                    rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                                    break;
                                                }
                                            }
                                            else
                                            {
                                                PRINT("bank state : " << bankStates[nextRankPRE][nextBankPRE].currentBankState);
                                                break;
                                            }
                                        }
                                        
                                        rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                        break;
                                    }
                                }
                            }
                            nextRankAndBank(nextRankPRE, nextBankPRE);
                        }
                        while (!(startingRank == nextRankPRE && startingBank == nextBankPRE));
                        
                        //if no PREs could be sent, just return false
                        if (!sendingPRE) return false;
                    }
                }
                else
                {
                    do // round robin over queues
                    {
                        vector<BusPacket *> &queue = getCommandQueue(nextRank,nextBank);
                        //make sure there is something there first
                        if (!queue.empty() && !((nextRank == refreshRank) && refreshWaiting))
                        {
                            //search from the beginning to find first issuable bus packet
                            for (size_t i=0;i<queue.size();i++)
                            {
                                BusPacket *packet = queue[i];
                                if (isIssuable(packet))
                                {
                                    //check for dependencies
                                    bool dependencyFound = false;
                                    for (size_t j=0;j<i;j++)
                                    {
                                        BusPacket *prevPacket = queue[j];
                                        if (prevPacket->busPacketType != ACTIVATE &&
                                            prevPacket->bank == packet->bank &&
                                            prevPacket->row == packet->row)
                                        {
                                            dependencyFound = true;
                                            break;
                                        }
                                    }
                                    if (dependencyFound) continue;
                                    
                                    *busPacket = packet;
                                    hit(*busPacket);
                                    bankAccess(*busPacket);
                                    
                                    // Oracle implementation
                                    if (ORACLE == true)
                                    {
                                        if (rowConflictHappened[(*busPacket)->rank][(*busPacket)->bank])
                                        {
                                            cycleOfRowConflict[(*busPacket)->rank][(*busPacket)->bank] = currentClockCycle;
                                            rowConflictHappened[(*busPacket)->rank][(*busPacket)->bank] = false;
                                        }
                                    }
                                    //if the bus packet before is an activate, that is the act that was
                                    //	paired with the column access we are removing, so we have to remove
                                    //	that activate as well (check i>0 because if i==0 then theres nothing before it)
                                    /*
                                    if (ENABLE_RESTORE)
                                    {
                                        if (packet->busPacketType == WRITE_RESTORE_PAGE)
                                        {
                                            PRINT("write restore is true");
                                            openPageRestoreDone[packet->rank][packet->bank] = true;
                                        }
                                        else
                                        {
                                            PRINT("write restore is false");
                                            openPageRestoreDone[packet->rank][packet->bank] = false;
                                        }
                                    }
                                    */
                                    if (i>0 && queue[i-1]->busPacketType == ACTIVATE)
                                    {
                                        rowAccessCounters[(*busPacket)->rank][(*busPacket)->bank]++;
                                        // i is being returned, but i-1 is being thrown away, so must delete it here
                                        delete (queue[i-1]);
                                        
                                        // remove both i-1 (the activate) and i (we've saved the pointer in *busPacket)
                                        queue.erase(queue.begin()+i-1,queue.begin()+i+1);
                                    }
                                    else // there's no activate before this packet
                                    {
                                        //or just remove the one bus packet
                                        queue.erase(queue.begin()+i);
                                    }
                                    foundIssuable = true;
                                    break;
                                    
                                }
                            }
                        }
                        
                        //if we found something, break out of do-while
                        if (foundIssuable)
                        {
                            break;
                        }
                        
                        //rank round robin
                        if (queuingStructure == PerRank)
                        {
                            nextRank = (nextRank + 1) % NUM_RANKS;
                            if (startingRank == nextRank)
                            {
                                break;
                            }
                        }
                        else
                        {
                            nextRankAndBank(nextRank, nextBank);
                            if (startingRank == nextRank && startingBank == nextBank)
                            {
                                break;
                            }
                        }
                    }
                    while (true);
                    
                    //if nothing was issuable, see if we can issue a PRE to an open bank
                    //	that has no other commands waiting
                    if (!foundIssuable)
                    {
                        //search for banks to close
                        bool sendingPRE = false;
                        unsigned startingRank = nextRankPRE;
                        unsigned startingBank = nextBankPRE;
                        
                        do // round robin over all ranks and banks
                        {
                            vector <BusPacket *> &queue = getCommandQueue(nextRankPRE, nextBankPRE);
                            bool found = false;
                            //check if bank is open
                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
                            {
                                if (ENABLE_RESTORE)
                                {
                                    for (size_t i=0;i<queue.size();i++)
                                    {
                                        //if there is something going to that bank and row, then we don't want to send a PRE
                                        if (queue[i]->bank == nextBankPRE &&
                                            queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
                                        {
                                            found = true;
                                            break;
                                        }
                                    }
                                }
                                else
                                {
                                    for (size_t i=0;i<queue.size();i++)
                                    {
                                        //if there is something going to that bank and row, then we don't want to send a PRE
                                        if (HYBRID_PAGE_POLICY_FLAG)
                                        {
                                            if (queue[i]->bank == nextBankPRE &&
                                                queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress && bankStates[nextRankPRE][nextBankPRE].lastCommand == ACTIVATE)
                                            {
                                                found = true;
                                                break;
                                            }
                                        }
                                        else
                                        {
                                            if (queue[i]->bank == nextBankPRE &&
                                                queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
                                            {
                                                found = true;
                                                break;
                                            }
                                        }
                                    }
                                }
                                
                                /*
                                 for (size_t i=0;i<queue.size();i++)
                                 {
                                 //if there is something going to that bank and row, then we don't want to send a PRE
                                 if (queue[i]->bank == nextBankPRE &&
                                 queue[i]->row == bankStates[nextRankPRE][nextBankPRE].openRowAddress)
                                 {
                                 found = true;
                                 break;
                                 }
                                 }
                                */
                                
                                //if nothing found going to that bank and row or too many accesses have happend, close it
                                if (!found || rowAccessCounters[nextRankPRE][nextBankPRE]==TOTAL_ROW_ACCESSES)
                                {
                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextPrecharge)
                                    {
                                        sendingPRE = true;
                                        //rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                        *busPacket = new BusPacket(PRECHARGE, 0, 0, 0, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                        if (ENABLE_RESTORE)
                                        {
                                            if (bankStates[nextRankPRE][nextBankPRE].currentBankState == RowActive)
                                            {
                                                
                                                if (openPageRestoreDone[nextRankPRE][nextBankPRE] == false)
                                                {
                                                    if (!found)// Oracle implementation
                                                    {
                                                        rowConflictHappened[nextRankPRE][nextBankPRE] = true;
                                                    }
                                                    
                                                    sendingPRE = false;
                                                    delete *busPacket;
                                                    
                                                    if (currentClockCycle >= bankStates[nextRankPRE][nextBankPRE].nextWrite)
                                                    {
                                                        *busPacket = new BusPacket(WRITE_RESTORE_PAGE, 0, 0, bankStates[nextRankPRE][nextBankPRE].openRowAddress, nextRankPRE, nextBankPRE, 0, dramsim_log);
                                                        sendingPRE = true;
                                                        openPageRestoreDone[nextRankPRE][nextBankPRE] =true;
                                                        break;
                                                    }
                                                }
                                                else
                                                {
                                                    openPageRestoreDone[nextRankPRE][nextBankPRE] = false;
                                                    rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                                    break;
                                                }
                                            }
                                            else
                                            {
                                                PRINT("bank state : " << bankStates[nextRankPRE][nextBankPRE].currentBankState);
                                                break;
                                            }
                                        }
                                        else
                                        {
                                            if (!found)
                                            {
                                                rowConflictHappened[nextRankPRE][nextBankPRE] = true;
                                            }
                                        }
                                        rowAccessCounters[nextRankPRE][nextBankPRE] = 0;
                                        break;
                                    }
                                }
                            }
                            nextRankAndBank(nextRankPRE, nextBankPRE);
                        }
                        while (!(startingRank == nextRankPRE && startingBank == nextBankPRE));
                        
                        //if no PREs could be sent, just return false
                        if (!sendingPRE) return false;
                    }
                }
            }
        }

    }

	//sendAct is flag used for posted-cas
	//  posted-cas is enabled when AL>0
	//  when sendAct is true, when don't want to increment our indexes
	//  so we send the column access that is paid with this act
	if (AL>0 && sendAct)
	{
		sendAct = false;
	}
	else
	{
		sendAct = true;
		nextRankAndBank(nextRank, nextBank);
	}

	//if its an activate, add a tfaw counter
	if ((*busPacket)->busPacketType==ACTIVATE)
	{
		tFAWCountdown[(*busPacket)->rank].push_back(tFAW);
	}
    
	return true;
}
//monitor the access rate of different banks
void CommandQueue::bankAccess(BusPacket *busPacket)
{
    if (busPacket->busPacketType == WRITE_P || busPacket->busPacketType == WRITE || busPacket->busPacketType == READ_P || busPacket->busPacketType == READ)
    {
        bankAccessCounters[busPacket->rank][busPacket->bank]++;
    }
}
//monitor the number of hits of the different banks
void CommandQueue::hit(BusPacket *busPacket)
{
    if (busPacket->busPacketType == WRITE_P || busPacket->busPacketType == WRITE || busPacket->busPacketType == READ_P || busPacket->busPacketType == READ)
    {
        if (previousPacketsArray[busPacket->rank][busPacket->bank] != NULL)
        {
            
            if (previousPacketsArray[busPacket->rank][busPacket->bank]->row == busPacket->row)
            {
                
                bankHitCounters[busPacket->rank][busPacket->bank]++;
                //bankAccessCounters[busPacket->rank][busPacket->bank]++;
                if (ENABLE_HYBRID_SATURATING_COUNTER == true)
                {
                    if (DISTRIBUTED_PAGE_POLICY_FLAG == false)
                    {
                        if (unifiedSaturatingCounter < 3)
                        {
                            unifiedSaturatingCounter++;
                        }
                        else
                        {
                            unifiedSaturatingCounter = 3;
                        }
                    }
                }
                else
                {
                    if (distributedSaturatingCounter[busPacket->rank][busPacket->bank] < 3)
                    {
                        distributedSaturatingCounter[busPacket->rank][busPacket->bank]++;
                    }
                    else
                    {
                        distributedSaturatingCounter[busPacket->rank][busPacket->bank] = 3;
                    }
                }
            }
            else
            {
                if (ENABLE_HYBRID_SATURATING_COUNTER == true)
                {
                    if (DISTRIBUTED_PAGE_POLICY_FLAG == false)
                    {
                        if (unifiedSaturatingCounter >0)
                        {
                            unifiedSaturatingCounter--;
                        }
                        else
                        {
                            unifiedSaturatingCounter = 0;
                        }
                    }
                    else
                    {
                        if (distributedSaturatingCounter[busPacket->rank][busPacket->bank] > 0)
                        {
                            distributedSaturatingCounter[busPacket->rank][busPacket->bank]--;
                        }
                        else
                        {
                            distributedSaturatingCounter[busPacket->rank][busPacket->bank] = 0;
                        }
                    }
                }
            }
            previousPacketsArray[busPacket->rank][busPacket->bank] = busPacket;
        }
        else
        {
            previousPacketsArray[busPacket->rank][busPacket->bank] = busPacket;
        }
    }
    if (ENABLE_HYBRID_SATURATING_COUNTER == true)
    {
        if (DISTRIBUTED_PAGE_POLICY_FLAG == false)
        {
            
            if (unifiedSaturatingCounter > 1)
            {
                rowBufferPolicy = OpenPage;
            }
            else
            {
                rowBufferPolicy = ClosePage;
            }
        }
        else
        {
            if (distributedSaturatingCounter[busPacket->rank][busPacket->bank] > 1)
            {
                bankRowBufferPolicy[busPacket->rank][busPacket->bank] = OpenPage;
                //PRINT("Row Buffer Policy of bank[" << busPacket->rank << "][" << busPacket->bank << "] is Open Page"   );
            }
            else
            {
                bankRowBufferPolicy[busPacket->rank][busPacket->bank] = ClosePage;
                //PRINT("Row Buffer Policy of bank[" << busPacket->rank << "][" << busPacket->bank << "] is Close Page"   );
            }
        }
    }

}

//check if a rank/bank queue has room for a certain number of bus packets
bool CommandQueue::hasRoomFor(unsigned numberToEnqueue, unsigned rank, unsigned bank)
{
    /*
    if (!enqueueFlag)
    {
        return false;
    }
     */
	vector<BusPacket *> &queue = getCommandQueue(rank, bank);
	return (CMD_QUEUE_DEPTH - queue.size() >= numberToEnqueue);
}

//prints the contents of the command queue
void CommandQueue::print()
{
	if (queuingStructure==PerRank)
	{
		PRINT(endl << "== Printing Per Rank Queue" );
		for (size_t i=0;i<NUM_RANKS;i++)
		{
			PRINT(" = Rank " << i << "  size : " << queues[i][0].size() );
			for (size_t j=0;j<queues[i][0].size();j++)
			{
				PRINTN("    "<< j << "]");
				queues[i][0][j]->print();
			}
		}
	}
	else if (queuingStructure==PerRankPerBank)
	{
		PRINT("\n== Printing Per Rank, Per Bank Queue" );

		for (size_t i=0;i<NUM_RANKS;i++)
		{
			PRINT(" = Rank " << i );
			for (size_t j=0;j<NUM_BANKS;j++)
			{
				PRINT("    Bank "<< j << "   size : " << queues[i][j].size() );

				for (size_t k=0;k<queues[i][j].size();k++)
				{
					PRINTN("       " << k << "]");
					queues[i][j][k]->print();
				}
			}
		}
	}
}

/** 
 * return a reference to the queue for a given rank, bank. Since we
 * don't always have a per bank queuing structure, sometimes the bank
 * argument is ignored (and the 0th index is returned 
 */
vector<BusPacket *> &CommandQueue::getCommandQueue(unsigned rank, unsigned bank)
{
	if (queuingStructure == PerRankPerBank)
	{
		return queues[rank][bank];
	}
	else if (queuingStructure == PerRank)
	{
		return queues[rank][0];
	}
	else
	{
		ERROR("Unknown queue structure");
		abort(); 
	}

}

//checks if busPacket is allowed to be issued
bool CommandQueue::isIssuable(BusPacket *busPacket)
{
    //PRINT("issueing packet: " << busPacket->busPacketType);
	switch (busPacket->busPacketType)
	{
	case REFRESH:

		break;
	case ACTIVATE:
        //PRINT("Activate rank/bank " << busPacket->rank << "/" << busPacket->bank);
        //PRINT("current bank state " << bankStates[busPacket->rank][busPacket->bank].currentBankState);
        //PRINT("last command " << bankStates[busPacket->rank][busPacket->bank].lastCommand);
        //PRINT("tfaw count down " << tFAWCountdown[busPacket->rank].size());
            //PRINT("command is activate");
            /*
        if (bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
        {
            PRINT("next activate is fine");
        }
             */
		if ((bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle ||
		        bankStates[busPacket->rank][busPacket->bank].currentBankState == Refreshing) &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextActivate &&
		        tFAWCountdown[busPacket->rank].size() < 4)
		{
            //PRINT("activate issued");
            //PRINT("activate issued bank" << busPacket->bank);
            //PRINT("last command = " << bankStates[busPacket->rank][busPacket->bank].lastCommand);
            //PRINT("current command = " << busPacket->busPacketType);
            //PRINT("current bank state in command queue = " << bankStates[busPacket->rank][busPacket->bank].currentBankState);
			return true;
		}
		else
		{
            /*
            //if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive && currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextPrecharge && bankStates[busPacket->rank][busPacket->bank].lastCommand == ACTIVATE)
            if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive && currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextPrecharge && bankStates[busPacket->rank][busPacket->bank].lastCommand == ACTIVATE)
            {
                //rowActiveForClosePagePolicy = true;
                rowActiveProblemForClosePagePolicy[busPacket->rank][busPacket->bank] = true;
            }
             */
			return false;
		}
		break;
	case WRITE:
	case WRITE_P:
            /*
        if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
        {
            rowIdleForClosePagePolicy = true;
        }
             */
        /*
        if (busPacket->busPacketType == WRITE && HYBRID_PAGE_POLICY_FLAG == true && DISTRIBUTED_PAGE_POLICY_FLAG == false && rowBufferPolicy == ClosePage && rowAccessCounters[busPacket->rank][busPacket->bank] == TOTAL_ROW_ACCESSES)
        {
            rowAccessCounters[busPacket->rank][busPacket->bank]--;
        }
        if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage)
        {
            rowAccessCounters[busPacket->rank][busPacket->bank] = 0;
        }
         */
		if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextWrite &&
		        busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress &&
		        rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES)
		{
            //PRINT("write packet is issued");
			return true;
		}
		else
		{
            /*
            if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
            
            else if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive && busPacket->row != bankStates[busPacket->rank][busPacket->bank].openRowAddress)
            {
                readWriteRowActiveProblemForClosePagePolicy[busPacket->rank][busPacket->bank] = true;
            }
             */
			return false;
		}
		break;
	case READ_P:
	case READ:
            //PRINT("Read rank/bank " << busPacket->rank << "/" << busPacket->bank);
            /*
             if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
             {
             rowIdleForClosePagePolicy = true;
             }
             */
            
            if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive)
            {
                //PRINT("row active");
                if (currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextRead)
                {
                    
                    if (busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress)
                    {
                        
                        if (rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES)
                        {
                            
                            return true;
                        }
                    }
                }
            }
        /*
		if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextRead &&
		        busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress &&
		        rowAccessCounters[busPacket->rank][busPacket->bank] < TOTAL_ROW_ACCESSES)
		{
			return true;
		}
        */
		else
		{
            /*
            if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
             */
            /*
            if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage && rowAccessCounters[busPacket->rank][busPacket->bank] >= TOTAL_ROW_ACCESSES)
            {
                rowAccessCounters[busPacket->rank][busPacket->bank] = 0;
            }
            if (bankRowBufferPolicy[busPacket->rank][busPacket->bank] == ClosePage)
            {
                //PRINT("Total row access = " << TOTAL_ROW_ACCESSES);
                //PRINT("row access counter = " << rowAccessCounters[busPacket->rank][busPacket->bank]);
                if (bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
                {
                    
                    //rowIdleForClosePagePolicy = true;
                    rowIdleProblemForClosePagePolicy[busPacket->rank][busPacket->bank] = true;
                }
                else if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive && busPacket->row != bankStates[busPacket->rank][busPacket->bank].openRowAddress)
                {
                    readWriteRowActiveProblemForClosePagePolicy[busPacket->rank][busPacket->bank] = true;
                }
            }
            else
            {
                if (bankStates[busPacket->rank][busPacket->bank].currentBankState == Idle)
                {
                    //rowIdleForClosePagePolicy = true;
                    rowIdleProblemForOpenPagePolicy[busPacket->rank][busPacket->bank] = true;
                }
            }
             */
			return false;
		}
		break;
	case PRECHARGE:
		if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
		        currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextPrecharge)
		{
			return true;
		}
		else
		{
			return false;
		}
		break;
    
    case WRITE_RESTORE_PAGE:
            if (bankStates[busPacket->rank][busPacket->bank].currentBankState == RowActive &&
                currentClockCycle >= bankStates[busPacket->rank][busPacket->bank].nextWrite &&
                busPacket->row == bankStates[busPacket->rank][busPacket->bank].openRowAddress)
            {
                //rowAccessCounters[busPacket->rank][busPacket->bank]--;
                return true;
            }
            else
            {
                return false;
            }
    
	default:
		ERROR("== Error - Trying to issue a crazy bus packet type : ");
		busPacket->print();
		exit(0);
	}
	return false;
}

//figures out if a rank's queue is empty
bool CommandQueue::isEmpty(unsigned rank)
{
	if (queuingStructure == PerRank)
	{
		return queues[rank][0].empty();
	}
	else if (queuingStructure == PerRankPerBank)
	{
		for (size_t i=0;i<NUM_BANKS;i++)
		{
			if (!queues[rank][i].empty()) return false;
		}
		return true;
	}
	else
	{
		DEBUG("Invalid Queueing Stucture");
		abort();
	}
}

//tells the command queue that a particular rank is in need of a refresh
void CommandQueue::needRefresh(unsigned rank)
{
	refreshWaiting = true;
	refreshRank = rank;
}

void CommandQueue::nextRankAndBank(unsigned &rank, unsigned &bank)
{
	if (schedulingPolicy == RankThenBankRoundRobin)
	{
		rank++;
		if (rank == NUM_RANKS)
		{
			rank = 0;
			bank++;
			if (bank == NUM_BANKS)
			{
				bank = 0;
			}
		}
	}
	//bank-then-rank round robin
	else if (schedulingPolicy == BankThenRankRoundRobin)
	{
		bank++;
		if (bank == NUM_BANKS)
		{
			bank = 0;
			rank++;
			if (rank == NUM_RANKS)
			{
				rank = 0;
			}
		}
	}
	else
	{
		ERROR("== Error - Unknown scheduling policy");
		exit(0);
	}

}

void CommandQueue::update()
{
	//do nothing since pop() is effectively update(),
	//needed for SimulatorObject
	//TODO: make CommandQueue not a SimulatorObject
}
