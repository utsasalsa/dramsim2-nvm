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



//MemoryController.cpp
//
//Class file for memory controller object
//

#include "MemoryController.h"
#include "MemorySystem.h"
#include "AddressMapping.h"

#define SEQUENTIAL(rank,bank) (rank*NUM_BANKS)+bank

/* Power computations are localized to MemoryController.cpp */
extern float IDD0;
extern float IDD1;
extern float IDD2P;
extern float IDD2Q;
extern float IDD2N;
extern float IDD3Pf;
extern float IDD3Ps;
extern float IDD3N;
extern float IDD4W;
extern float IDD4R;
extern float IDD5;
extern float IDD6;
extern float IDD6L;
extern float IDD7;
extern float Vdd;

using namespace DRAMSim;

MemoryController::MemoryController(MemorySystem *parent, CSVWriter &csvOut_, ostream &dramsim_log_) :
dramsim_log(dramsim_log_),
bankStates(NUM_RANKS, vector<BankState>(NUM_BANKS, dramsim_log)),
commandQueue(bankStates, dramsim_log_),
poppedBusPacket(NULL),
csvOut(csvOut_),
totalTransactions(0),
refreshRank(0),
prechargeFlag(false),
unifiedNumberOfOpenPageSwitching(0),
unifiedNumberOfClosePageSwitching(0),
totalClosePageTransactions(0),
totalOpenPageTransactions(0)
{
    //get handle on parent
    parentMemorySystem = parent;
    
    
    //bus related fields
    outgoingCmdPacket = NULL;
    outgoingDataPacket = NULL;
    dataCyclesLeft = 0;
    cmdCyclesLeft = 0;
    
    //set here to avoid compile errors
    currentClockCycle = 0;
    
    //reserve memory for vectors
    transactionQueue.reserve(TRANS_QUEUE_DEPTH);
    powerDown = vector<bool>(NUM_RANKS,false);
    grandTotalBankAccesses = vector<uint64_t>(NUM_RANKS*NUM_BANKS,0);
    totalReadsPerBank = vector<uint64_t>(NUM_RANKS*NUM_BANKS,0);
    totalWritesPerBank = vector<uint64_t>(NUM_RANKS*NUM_BANKS,0);
    totalReadsPerRank = vector<uint64_t>(NUM_RANKS,0);
    totalWritesPerRank = vector<uint64_t>(NUM_RANKS,0);
    
    writeDataCountdown.reserve(NUM_RANKS);
    writeDataToSend.reserve(NUM_RANKS);
    refreshCountdown.reserve(NUM_RANKS);
    
    //Power related packets
    backgroundEnergy = vector <double >(NUM_RANKS,0);
    burstEnergy = vector <double> (NUM_RANKS,0);
    actpreEnergy = vector <double> (NUM_RANKS,0);
    refreshEnergy = vector <double> (NUM_RANKS,0);
    
    totalEpochLatency = vector<uint64_t> (NUM_RANKS*NUM_BANKS,0);
    
    //staggers when each rank is due for a refresh
    for (size_t i=0;i<NUM_RANKS;i++)
    {
        refreshCountdown.push_back((int)((REFRESH_PERIOD/tCK)/NUM_RANKS)*(i+1));
    }
    
    distributedNumberOfOpenPageSwitching = vector< vector<unsigned> > (NUM_RANKS, vector<unsigned> (NUM_BANKS, 0));
    distributedNumberOfClosePageSwitching = vector< vector<unsigned> > (NUM_RANKS, vector<unsigned> (NUM_BANKS, 0));
    
    lastTransactionAddressArray = vector< vector<uint64_t> > (NUM_RANKS, vector<uint64_t> (NUM_BANKS, 0));
    writeRestoreDoneForOpenPage = vector< vector<bool> > (NUM_RANKS, vector<bool> (NUM_BANKS, false));
    firstRankBankTransaction = vector< vector<bool> > (NUM_RANKS, vector<bool> (NUM_BANKS, true));
    
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket *bpacket)
{
    if (bpacket->busPacketType != DATA)
    {
        ERROR("== Error - Memory Controller received a non-DATA bus packet from rank");
        bpacket->print();
        exit(0);
    }
    
    if (DEBUG_BUS)
    {
        PRINTN(" -- MC Receiving From Data Bus : ");
        bpacket->print();
    }
    
    //add to return read data queue
    returnTransaction.push_back(new Transaction(RETURN_DATA, bpacket->physicalAddress, bpacket->data));
    totalReadsPerBank[SEQUENTIAL(bpacket->rank,bpacket->bank)]++;
    
    // this delete statement saves a mindboggling amount of memory
    delete(bpacket);
}

//sends read data back to the CPU
void MemoryController::returnReadData(const Transaction *trans)
{
    if (parentMemorySystem->ReturnReadData!=NULL)
    {
        (*parentMemorySystem->ReturnReadData)(parentMemorySystem->systemID, trans->address, currentClockCycle);
    }
}

//gives the memory controller a handle on the rank objects
void MemoryController::attachRanks(vector<Rank *> *ranks)
{
    this->ranks = ranks;
}

//memory controller update
void MemoryController::update()
{
    
    //PRINT(" ------------------------- [" << currentClockCycle << "] -------------------------");
    
    //update bank states
    for (size_t i=0;i<NUM_RANKS;i++)
    {
        for (size_t j=0;j<NUM_BANKS;j++)
        {
            if (bankStates[i][j].stateChangeCountdown>0)
            {
                //decrement counters
                bankStates[i][j].stateChangeCountdown--;
                
                //if counter has reached 0, change state
                if (bankStates[i][j].stateChangeCountdown == 0)
                {
                    switch (bankStates[i][j].lastCommand)
                    {
                        //only these commands have an implicit state change
                        case WRITE_P:
                        case READ_P:
                        
                            if (ENABLE_RESTORE == true)
                            {
                                if (bankStates[i][j].lastCommand == WRITE_P)
                                {
                                    bankStates[i][j].currentBankState = Precharging;
                                    bankStates[i][j].lastCommand = PRECHARGE;
                                    bankStates[i][j].stateChangeCountdown = tRP;
                                }
                            }
                            else
                            {
                                bankStates[i][j].currentBankState = Precharging;
                                bankStates[i][j].lastCommand = PRECHARGE;
                                bankStates[i][j].stateChangeCountdown = tRP;
                            }
                            break;
                        /*
                        case READ:
                        case WRITE:
                        //case WRITE_RESTORE_PAGE:
                            if (HYBRID_PAGE_POLICY_FLAG == true && ENABLE_RESTORE == false && rowBufferPolicy == ClosePage)
                            {
                                //if (DISTRIBUTED_PAGE_POLICY_FLAG == false && rowBufferPolicy == ClosePage)
                                //{
                                    bankStates[i][j].currentBankState = Precharging;
                                    bankStates[i][j].lastCommand = PRECHARGE;
                                    bankStates[i][j].stateChangeCountdown = tRP;
                                //}
                            }
                        */
                        case REFRESH:
                        case PRECHARGE:
                            bankStates[i][j].currentBankState = Idle;
                            break;
                        default:
                            break;
                    }
                }
            }
        }
    }
    
    
    //check for outgoing command packets and handle countdowns
    if (outgoingCmdPacket != NULL)
    {
        cmdCyclesLeft--;
        if (cmdCyclesLeft == 0) //packet is ready to be received by rank
        {
            //(*ranks)[outgoingCmdPacket->rank]->receiveFromBus(outgoingCmdPacket);
            outgoingCmdPacket = NULL;
        }
    }
    
    //check for outgoing data packets and handle countdowns
    if (outgoingDataPacket != NULL)
    {
        dataCyclesLeft--;
        if (dataCyclesLeft == 0)
        {
            //inform upper levels that a write is done
            if (parentMemorySystem->WriteDataDone!=NULL)
            {
                (*parentMemorySystem->WriteDataDone)(parentMemorySystem->systemID,outgoingDataPacket->physicalAddress, currentClockCycle);
            }
            //(*ranks)[outgoingDataPacket->rank]->receiveFromBus(outgoingDataPacket);
            outgoingDataPacket=NULL;
        }
    }
    
    
    //if any outstanding write data needs to be sent
    //and the appropriate amount of time has passed (WL)
    //then send data on bus
    //
    //write data held in fifo vector along with countdowns
    if (writeDataCountdown.size() > 0)
    {
        for (size_t i=0;i<writeDataCountdown.size();i++)
        {
            writeDataCountdown[i]--;
        }
        
        if (writeDataCountdown[0]==0)
        {
            //send to bus and print debug stuff
            if (DEBUG_BUS)
            {
                PRINTN(" -- MC Issuing On Data Bus    : ");
                writeDataToSend[0]->print();
            }
            
            // queue up the packet to be sent
            if (outgoingDataPacket != NULL)
            {
                ERROR("== Error - Data Bus Collision");
                exit(-1);
            }
            
            outgoingDataPacket = writeDataToSend[0];
            dataCyclesLeft = BL/2;
            
            totalTransactions++;
            totalWritesPerBank[SEQUENTIAL(writeDataToSend[0]->rank,writeDataToSend[0]->bank)]++;
            
            writeDataCountdown.erase(writeDataCountdown.begin());
            writeDataToSend.erase(writeDataToSend.begin());
        }
    }
    
    //if its time for a refresh issue a refresh
    // else pop from command queue if it's not empty
    if (refreshCountdown[refreshRank]==0)
    {
        commandQueue.needRefresh(refreshRank);
        (*ranks)[refreshRank]->refreshWaiting = true;
        refreshCountdown[refreshRank] =	 REFRESH_PERIOD/tCK;
        refreshRank++;
        if (refreshRank == NUM_RANKS)
        {
            refreshRank = 0;
        }
    }
    //if a rank is powered down, make sure we power it up in time for a refresh
    else if (powerDown[refreshRank] && refreshCountdown[refreshRank] <= tXP)
    {
        (*ranks)[refreshRank]->refreshWaiting = true;
    }
    
    //pass a pointer to a poppedBusPacket
    
    //function returns true if there is something valid in poppedBusPacket
    if (commandQueue.pop(&poppedBusPacket))
    {
        //PRINT("current packet type = " << poppedBusPacket->busPacketType);
        packetType = poppedBusPacket->busPacketType;
        if (poppedBusPacket->busPacketType == WRITE || poppedBusPacket->busPacketType == WRITE_P)
        {
            
            writeDataToSend.push_back(new BusPacket(DATA, poppedBusPacket->physicalAddress, poppedBusPacket->column,
                                                    poppedBusPacket->row, poppedBusPacket->rank, poppedBusPacket->bank,
                                                    poppedBusPacket->data, dramsim_log));
            writeDataCountdown.push_back(WL);
        }
        //Count the fraction of open page and close page transactions
        if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == OpenPage)
        {
            totalOpenPageTransactions++;
        }
        else if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == ClosePage)
        {
            totalClosePageTransactions++;
        }
        //
        //update each bank's state based on the command that was just popped out of the command queue
        //
        //for readability's sake
        unsigned rank = poppedBusPacket->rank;
        unsigned bank = poppedBusPacket->bank;
        switch (poppedBusPacket->busPacketType)
        {
            case READ_P:
            case READ:
                //add energy to account for total
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding Read energy to total energy");
                }
                burstEnergy[rank] += (IDD4R - IDD3N) * BL/2 * NUM_DEVICES;
                if (HYBRID_PAGE_POLICY_FLAG == false || DISTRIBUTED_PAGE_POLICY_FLAG == false)
                {
                    if (poppedBusPacket->busPacketType == READ_P)
                    {
                        //Don't bother setting next read or write times because the bank is no longer active
                        bankStates[rank][bank].nextActivate = max(currentClockCycle + READ_AUTOPRE_DELAY,
                                                                  bankStates[rank][bank].nextActivate);
                        bankStates[rank][bank].lastCommand = READ_P;
                        bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;
                    }
                    else if (poppedBusPacket->busPacketType == READ)
                    {
                        bankStates[rank][bank].nextPrecharge = max(currentClockCycle + READ_TO_PRE_DELAY,
                                                                   bankStates[rank][bank].nextPrecharge);
                        bankStates[rank][bank].lastCommand = READ;
                        
                    }
                }
                else
                {
                    if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == ClosePage)
                    {
                        //Don't bother setting next read or write times because the bank is no longer active
                        bankStates[rank][bank].nextActivate = max(currentClockCycle + READ_AUTOPRE_DELAY,
                                                                  bankStates[rank][bank].nextActivate);
                        bankStates[rank][bank].lastCommand = READ_P;
                        bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;
                    }
                    else if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == OpenPage)
                    {
                        bankStates[rank][bank].nextPrecharge = max(currentClockCycle + READ_TO_PRE_DELAY,
                                                                   bankStates[rank][bank].nextPrecharge);
                        bankStates[rank][bank].lastCommand = READ;
                        
                    }
                }
                
                for (size_t i=0;i<NUM_RANKS;i++)
                {
                    for (size_t j=0;j<NUM_BANKS;j++)
                    {
                        if (i!=poppedBusPacket->rank)
                        {
                            //check to make sure it is active before trying to set (save's time?)
                            if (bankStates[i][j].currentBankState == RowActive)
                            {
                                bankStates[i][j].nextRead = max(currentClockCycle + BL/2 + tRTRS, bankStates[i][j].nextRead);
                                bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                                                                 bankStates[i][j].nextWrite);
                            }
                        }
                        else
                        {
                            bankStates[i][j].nextRead = max(currentClockCycle + max(tCCD, BL/2), bankStates[i][j].nextRead);
                            bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
                                                             bankStates[i][j].nextWrite);
                        }
                    }
                }
                
                if (HYBRID_PAGE_POLICY_FLAG == false || DISTRIBUTED_PAGE_POLICY_FLAG == false)
                {
                    if (poppedBusPacket->busPacketType == READ_P)
                    {
                        bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
                        bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
                    }
                }
                else
                {
                    if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == ClosePage)
                    {
                        //set read and write to nextActivate so the state table will prevent a read or write
                        //  being issued (in cq.isIssuable())before the bank state has been changed because of the
                        //  auto-precharge associated with this command
                        bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
                        bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
                    }
                }
                
                /*
                if (poppedBusPacket->busPacketType == READ_P)
                {
                    bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
                    bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
                }
                 */
                
                if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == ClosePage)
                {
                    //set read and write to nextActivate so the state table will prevent a read or write
                    //  being issued (in cq.isIssuable())before the bank state has been changed because of the
                    //  auto-precharge associated with this command
                    bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
                    bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
                }
                
                break;
            case WRITE_P:
            case WRITE:
                if (HYBRID_PAGE_POLICY_FLAG == false || DISTRIBUTED_PAGE_POLICY_FLAG == false)
                {
                    if (poppedBusPacket->busPacketType == WRITE_P)
                    {
                        bankStates[rank][bank].nextActivate = max(currentClockCycle + WRITE_AUTOPRE_DELAY,
                                                                  bankStates[rank][bank].nextActivate);
                        bankStates[rank][bank].lastCommand = WRITE_P;
                        bankStates[rank][bank].stateChangeCountdown = WRITE_TO_PRE_DELAY;
                    }
                    else if (poppedBusPacket->busPacketType == WRITE)
                    {
                        bankStates[rank][bank].nextPrecharge = max(currentClockCycle + WRITE_TO_PRE_DELAY,
                                                                   bankStates[rank][bank].nextPrecharge);
                        bankStates[rank][bank].lastCommand = WRITE;
                    }
                    
                }
                else
                {
                    if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == ClosePage)
                    {
                        bankStates[rank][bank].nextActivate = max(currentClockCycle + WRITE_AUTOPRE_DELAY,
                                                                  bankStates[rank][bank].nextActivate);
                        bankStates[rank][bank].lastCommand = WRITE_P;
                        bankStates[rank][bank].stateChangeCountdown = WRITE_TO_PRE_DELAY;
                    }
                    else if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == OpenPage)
                        
                    {
                        bankStates[rank][bank].nextPrecharge = max(currentClockCycle + WRITE_TO_PRE_DELAY,
                                                                   bankStates[rank][bank].nextPrecharge);
                        bankStates[rank][bank].lastCommand = WRITE;
                    }
                }
                
                
                //add energy to account for total
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding Write energy to total energy");
                }
                burstEnergy[rank] += (IDD4W - IDD3N) * BL/2 * NUM_DEVICES;
                
                for (size_t i=0;i<NUM_RANKS;i++)
                {
                    for (size_t j=0;j<NUM_BANKS;j++)
                    {
                        if (i!=poppedBusPacket->rank)
                        {
                            if (bankStates[i][j].currentBankState == RowActive)
                            {
                                bankStates[i][j].nextWrite = max(currentClockCycle + BL/2 + tRTRS, bankStates[i][j].nextWrite);
                                bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_R,
                                                                bankStates[i][j].nextRead);
                            }
                        }
                        else
                        {
                            bankStates[i][j].nextWrite = max(currentClockCycle + max(BL/2, tCCD), bankStates[i][j].nextWrite);
                            bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_B,
                                                            bankStates[i][j].nextRead);
                        }
                    }
                }
                
                //set read and write to nextActivate so the state table will prevent a read or write
                //  being issued (in cq.isIssuable())before the bank state has been changed because of the
                //  auto-precharge associated with this command
                if (HYBRID_PAGE_POLICY_FLAG == false || DISTRIBUTED_PAGE_POLICY_FLAG == false)
                {
                    if (poppedBusPacket->busPacketType == WRITE_P)                        
                    {
                        bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
                        bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
                    }
                }
                else
                {
                    if (commandQueue.bankRowBufferPolicy[poppedBusPacket->rank][poppedBusPacket->bank] == ClosePage)
                    
                    {
                        bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
                        bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
                    }
                }

                
                break;
            ///////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////
            case WRITE_RESTORE_PAGE:
                bankStates[rank][bank].nextPrecharge = max(currentClockCycle + RESTORE_PAGE,
                                                           bankStates[rank][bank].nextPrecharge);
                bankStates[rank][bank].lastCommand = WRITE_RESTORE_PAGE;
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding Write energy to total energy");
                }
                burstEnergy[rank] += (IDD4W - IDD3N) * BL * NUM_DEVICES;
                
                
                for (size_t i=0;i<NUM_RANKS;i++)
                {
                    for (size_t j=0;j<NUM_BANKS;j++)
                    {
                        if (i!=poppedBusPacket->rank)
                        {
                            if (bankStates[i][j].currentBankState == RowActive)
                            {
                                bankStates[i][j].nextWrite = max(currentClockCycle + BL/2 + tRTRS, bankStates[i][j].nextWrite);
                                bankStates[i][j].nextRead = max(currentClockCycle +  WRITE_TO_READ_DELAY_R,
                                                                bankStates[i][j].nextRead);
                            }
                        }
                        else
                        {
                            bankStates[i][j].nextWrite = max(currentClockCycle + max(BL/2, tCCD), bankStates[i][j].nextWrite);
                            bankStates[i][j].nextRead = max(currentClockCycle +  WRITE_TO_READ_DELAY_B,
                                                            bankStates[i][j].nextRead);
                        }
                    }
                }

            ///////////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////////
            case ACTIVATE:
                //add energy to account for total
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding Activate and Precharge energy to total energy");
                }
                actpreEnergy[rank] += ((IDD0 * tRC) - ((IDD3N * tRAS) + (IDD2N * (tRC - tRAS)))) * NUM_DEVICES;
                
                bankStates[rank][bank].currentBankState = RowActive;
                bankStates[rank][bank].lastCommand = ACTIVATE;
                bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
                bankStates[rank][bank].nextActivate = max(currentClockCycle + tRC, bankStates[rank][bank].nextActivate);
                bankStates[rank][bank].nextPrecharge = max(currentClockCycle + tRAS, bankStates[rank][bank].nextPrecharge);
                
                //if we are using posted-CAS, the next column access can be sooner than normal operation
                
                bankStates[rank][bank].nextRead = max(currentClockCycle + (tRCD-AL), bankStates[rank][bank].nextRead);
                bankStates[rank][bank].nextWrite = max(currentClockCycle + (tRCD-AL), bankStates[rank][bank].nextWrite);
                
                for (size_t i=0;i<NUM_BANKS;i++)
                {
                    if (i!=poppedBusPacket->bank)
                    {
                        bankStates[rank][i].nextActivate = max(currentClockCycle + tRRD, bankStates[rank][i].nextActivate);
                    }
                }
                
                break;
            case PRECHARGE:
                bankStates[rank][bank].currentBankState = Precharging;
                bankStates[rank][bank].lastCommand = PRECHARGE;
                bankStates[rank][bank].stateChangeCountdown = tRP;
                bankStates[rank][bank].nextActivate = max(currentClockCycle + tRP, bankStates[rank][bank].nextActivate);
                
                break;
            case REFRESH:
                //add energy to account for total
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding Refresh energy to total energy");
                }
                refreshEnergy[rank] += (IDD5 - IDD3N) * tRFC * NUM_DEVICES;
                
                for (size_t i=0;i<NUM_BANKS;i++)
                {
                    bankStates[rank][i].nextActivate = currentClockCycle + tRFC;
                    bankStates[rank][i].currentBankState = Refreshing;
                    bankStates[rank][i].lastCommand = REFRESH;
                    bankStates[rank][i].stateChangeCountdown = tRFC;
                }
                
                break;
            default:
                ERROR("== Error - Popped a command we shouldn't have of type : " << poppedBusPacket->busPacketType);
                exit(0);
        }
        
        //issue on bus and print debug
        if (DEBUG_BUS)
        {
            PRINTN(" -- MC Issuing On Command Bus : ");
            poppedBusPacket->print();
        }
        
        //check for collision on bus
        if (outgoingCmdPacket != NULL)
        {
            ERROR("== Error - Command Bus Collision");
            exit(-1);
        }
        outgoingCmdPacket = poppedBusPacket;
        cmdCyclesLeft = tCMD;
        
    }
    
    for (size_t i=0;i<transactionQueue.size();i++)
    {
        //pop off top transaction from queue
        //
        //	assuming simple scheduling at the moment
        //	will eventually add policies here
        Transaction *transaction = transactionQueue[i];
        
        //map address to rank,bank,row,col
        unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn;
        
        // pass these in as references so they get set by the addressMapping function
        addressMapping(transaction->address, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn);
        
        //if we have room, break up the transaction into the appropriate commands
        //and add them to the command queue
        if (commandQueue.hasRoomFor(2, newTransactionRank, newTransactionBank))
        {
            if (DEBUG_ADDR_MAP)
            {
                PRINTN("== New Transaction - Mapping Address [0x" << hex << transaction->address << dec << "]");
                if (transaction->transactionType == DATA_READ)
                {
                    PRINT(" (Read)");
                }
                else
                {
                    PRINT(" (Write)");
                }
                PRINT("  Rank : " << newTransactionRank);
                PRINT("  Bank : " << newTransactionBank);
                PRINT("  Row  : " << newTransactionRow);
                PRINT("  Col  : " << newTransactionColumn);
            }
            
            
            
            //now that we know there is room in the command queue, we can remove from the transaction queue
            /*
            if (ENABLE_RESTORE == true)
            {
                unsigned lastTransactionChan, lastTransactionRank, lastTransactionBank, lastTransactionRow, lastTransactionColumn;
                uint64_t lastTransactionAddress = lastTransactionAddressArray[newTransactionRank][newTransactionBank];
                
                addressMapping(lastTransactionAddress, lastTransactionChan, lastTransactionRank, lastTransactionBank, lastTransactionRow, lastTransactionColumn);
                
                
                if (newTransactionRow != lastTransactionRow && writeRestoreDoneForOpenPage[newTransactionRank][newTransactionBank] == false && rowBufferPolicy == OpenPage && firstRankBankTransaction[newTransactionRank][newTransactionBank] == false)
                {
                    transaction = new Transaction(DATA_PAGE_RESTORE, lastTransactionAddress, 0);
                    writeRestoreDoneForOpenPage[newTransactionRank][newTransactionBank] = true;
                    addressMapping(lastTransactionAddress, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn);

                }
                else
                {
                    firstRankBankTransaction[newTransactionRank][newTransactionBank] = false;
                    transactionQueue.erase(transactionQueue.begin()+i);
                    writeRestoreDoneForOpenPage[newTransactionRank][newTransactionBank] = false;
                    lastTransactionAddressArray[newTransactionRank][newTransactionBank] = transaction->address;
                }
            }
            else
            {
                transactionQueue.erase(transactionQueue.begin()+i);
            }
            */
            transactionQueue.erase(transactionQueue.begin()+i);
            //create activate command to the row we just translated
            
            BusPacket *ACTcommand = new BusPacket(ACTIVATE, transaction->address,
                                                  newTransactionColumn, newTransactionRow, newTransactionRank,
                                                  newTransactionBank, 0, dramsim_log);
            
            //create read or write command and enqueue it
            //BusPacketType bpType = transaction->getBusPacketType();
            
            BusPacketType bpType;
            if (HYBRID_PAGE_POLICY_FLAG == true)
            {
                bpType = transaction->getBusPacketType(transaction->address, commandQueue.bankRowBufferPolicy[newTransactionRank][newTransactionBank]);
            }
            else
            {
                bpType = transaction->getBusPacketType();
            }
            
            BusPacket *command = new BusPacket(bpType, transaction->address,
                                               newTransactionColumn, newTransactionRow, newTransactionRank,
                                               newTransactionBank, transaction->data, dramsim_log);
            
            if (transaction->restoreWrite == true && (command->busPacketType == WRITE || command->busPacketType == WRITE_P))
            {
                //PRINT("restore command added");
                command->restoreWrite = true;
            }
            /*
            if (ENABLE_RESTORE)
            {
                if (command->busPacketType == WRITE_P && command->restoreWrite)
                {
                    delete ACTcommand;
                    commandQueue.enqueue(command);
                }
                else if (command->busPacketType == WRITE && command->restoreWrite)
                {
                    delete ACTcommand;
                    delete command;
                    //commandQueue.enqueue(ACTcommand);
                    //commandQueue.enqueue(command);
                }
                else
                {
                    commandQueue.enqueue(ACTcommand);
                    commandQueue.enqueue(command);
                }
            }
            else
            {
                commandQueue.enqueue(ACTcommand);
                commandQueue.enqueue(command);
            }
            */
            if (ENABLE_RESTORE)
            {
                if (command->busPacketType == WRITE_P && command->restoreWrite)
                {
                    delete ACTcommand;
                    commandQueue.enqueue(command);
                }
                else if (command->busPacketType == WRITE && command->restoreWrite)
                {
                    delete ACTcommand;
                    delete command;
                    //commandQueue.enqueue(ACTcommand);
                    //commandQueue.enqueue(command);
                }
                else
                {
                    commandQueue.enqueue(ACTcommand);
                    commandQueue.enqueue(command);
                }
            }
            else
            {
                commandQueue.enqueue(ACTcommand);
                commandQueue.enqueue(command);
            }
            // If we have a read, save the transaction so when the data comes back
            // in a bus packet, we can staple it back into a transaction and return it
            if (transaction->transactionType == DATA_READ)
            {
                pendingReadTransactions.push_back(transaction);
            }
            else
            {
                // just delete the transaction now that it's a buspacket
                delete transaction;
            }
            /* only allow one transaction to be scheduled per cycle -- this should
             * be a reasonable assumption considering how much logic would be
             * required to schedule multiple entries per cycle (parallel data
             * lines, switching logic, decision logic)
             */
            break;
        }
        else // no room, do nothing this cycle
        {
            //PRINT( "== Warning - No room in command queue" << endl;
        }
    }
    
    
    //calculate power
    //  this is done on a per-rank basis, since power characterization is done per device (not per bank)
    for (size_t i=0;i<NUM_RANKS;i++)
    {
        if (USE_LOW_POWER)
        {
            //if there are no commands in the queue and that particular rank is not waiting for a refresh...
            if (commandQueue.isEmpty(i) && !(*ranks)[i]->refreshWaiting)
            {
                //check to make sure all banks are idle
                bool allIdle = true;
                for (size_t j=0;j<NUM_BANKS;j++)
                {
                    if (bankStates[i][j].currentBankState != Idle)
                    {
                        allIdle = false;
                        break;
                    }
                }
                
                //if they ARE all idle, put in power down mode and set appropriate fields
                if (allIdle)
                {
                    powerDown[i] = true;
                    (*ranks)[i]->powerDown();
                    for (size_t j=0;j<NUM_BANKS;j++)
                    {
                        bankStates[i][j].currentBankState = PowerDown;
                        bankStates[i][j].nextPowerUp = currentClockCycle + tCKE;
                    }
                }
            }
            //if there IS something in the queue or there IS a refresh waiting (and we can power up), do it
            else if (currentClockCycle >= bankStates[i][0].nextPowerUp && powerDown[i]) //use 0 since theyre all the same
            {
                powerDown[i] = false;
                (*ranks)[i]->powerUp();
                for (size_t j=0;j<NUM_BANKS;j++)
                {
                    bankStates[i][j].currentBankState = Idle;
                    bankStates[i][j].nextActivate = currentClockCycle + tXP;
                }
            }
        }
        
        //check for open bank
        bool bankOpen = false;
        for (size_t j=0;j<NUM_BANKS;j++)
        {
            if (bankStates[i][j].currentBankState == Refreshing ||
                bankStates[i][j].currentBankState == RowActive)
            {
                bankOpen = true;
                break;
            }
        }
        
        //background power is dependent on whether or not a bank is open or not
        if (bankOpen)
        {
            if (DEBUG_POWER)
            {
                PRINT(" ++ Adding IDD3N to total energy [from rank "<< i <<"]");
            }
            backgroundEnergy[i] += IDD3N * NUM_DEVICES;
        }
        else
        {
            //if we're in power-down mode, use the correct current
            if (powerDown[i])
            {
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding IDD2P to total energy [from rank " << i << "]");
                }
                backgroundEnergy[i] += IDD2P * NUM_DEVICES;
            }
            else
            {
                if (DEBUG_POWER)
                {
                    PRINT(" ++ Adding IDD2N to total energy [from rank " << i << "]");
                }
                backgroundEnergy[i] += IDD2N * NUM_DEVICES;
            }
        }
    }
    
    //check for outstanding data to return to the CPU
    if (returnTransaction.size()>0)
    {
        if (DEBUG_BUS)
        {
            PRINTN(" -- MC Issuing to CPU bus : " << *returnTransaction[0]);
        }
        totalTransactions++;
        
        bool foundMatch=false;
        //find the pending read transaction to calculate latency
        for (size_t i=0;i<pendingReadTransactions.size();i++)
        {
            if (pendingReadTransactions[i]->address == returnTransaction[0]->address)
            {
                //if(currentClockCycle - pendingReadTransactions[i]->timeAdded > 2000)
                //	{
                //		pendingReadTransactions[i]->print();
                //		exit(0);
                //	}
                unsigned chan,rank,bank,row,col;
                addressMapping(returnTransaction[0]->address,chan,rank,bank,row,col);
                insertHistogram(currentClockCycle-pendingReadTransactions[i]->timeAdded,rank,bank);
                //return latency
                returnReadData(pendingReadTransactions[i]);
                
                delete pendingReadTransactions[i];
                pendingReadTransactions.erase(pendingReadTransactions.begin()+i);
                foundMatch=true;
                break;
            }
        }
        if (!foundMatch)
        {
            ERROR("Can't find a matching transaction for 0x"<<hex<<returnTransaction[0]->address<<dec);
            abort();
        }
        delete returnTransaction[0];
        returnTransaction.erase(returnTransaction.begin());
    }
    
    //decrement refresh counters
    for (size_t i=0;i<NUM_RANKS;i++)
    {
        refreshCountdown[i]--;
    }
    
    //
    //print debug
    //
    if (DEBUG_TRANS_Q)
    {
        PRINT("== Printing transaction queue");
        for (size_t i=0;i<transactionQueue.size();i++)
        {
            PRINTN("  " << i << "] "<< *transactionQueue[i]);
        }
    }
    
    if (DEBUG_BANKSTATE)
    {
        //TODO: move this to BankState.cpp
        PRINT("== Printing bank states (According to MC)");
        for (size_t i=0;i<NUM_RANKS;i++)
        {
            for (size_t j=0;j<NUM_BANKS;j++)
            {
                if (bankStates[i][j].currentBankState == RowActive)
                {
                    PRINTN("[" << bankStates[i][j].openRowAddress << "] ");
                }
                else if (bankStates[i][j].currentBankState == Idle)
                {
                    PRINTN("[idle] ");
                }
                else if (bankStates[i][j].currentBankState == Precharging)
                {
                    PRINTN("[pre] ");
                }
                else if (bankStates[i][j].currentBankState == Refreshing)
                {
                    PRINTN("[ref] ");
                }
                else if (bankStates[i][j].currentBankState == PowerDown)
                {
                    PRINTN("[lowp] ");
                }
            }
            PRINT(""); // effectively just cout<<endl;
        }
    }
    
    if (DEBUG_CMD_Q)
    {
        commandQueue.print();
    }
    
    commandQueue.step();
    
}

bool MemoryController::WillAcceptTransaction()
{
    return transactionQueue.size() < TRANS_QUEUE_DEPTH;
}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans)
{
    if (WillAcceptTransaction())
    {
        trans->timeAdded = currentClockCycle;
        transactionQueue.push_back(trans);
        return true;
    }
    else
    {
        return false;
    }
}

void MemoryController::resetStats()
{
    //double threshold = ((double)(tRP) / (double)(tRP + tRCD));
    //threshold = 0;
    if (HYBRID_PAGE_POLICY_FLAG == true)
    {
        //double threshold = ((double)(tRP + RESTORE_PAGE - RESTORE_LINE) / (double)(tRP + tRCD + RESTORE_PAGE));
        double threshold = 0.1;
        //double threshold = 0.01;
        //PRINT("Total open page transactions = " << totalOpenPageTransactions);
        //PRINT("Total close page transactions = " << totalClosePageTransactions);
        if (DISTRIBUTED_PAGE_POLICY_FLAG)
        {
            PRINT("Threshold = " << threshold);
            for (size_t i=0; i<NUM_RANKS; i++)
            {                
                for (size_t j=0; j<NUM_BANKS; j++)
                {
                    //PRINT("commandQueue.bankHitCounters = " << (double)commandQueue.bankHitCounters[i][j]);
                    //PRINT("commandQueue.bankAccessCounters = " << (double)commandQueue.bankAccessCounters[i][j]);
                    /*
                    PRINT("Current bank states "<<bankStates[i][j].currentBankState);
                    PRINT("last command = " << bankStates[i][j].lastCommand);
                    PRINT("packet row address: " << poppedBusPacket->row);
                    PRINT("bank open row address: " << bankStates[i][j].openRowAddress);
                    PRINT("current command = " << packetType);
                    */
                    double hitRate = (double)commandQueue.bankHitCounters[i][j] / (double)commandQueue.bankAccessCounters[i][j];
                    
                    
                    if (commandQueue.bankAccessCounters[i][j] >0)
                    {
                        //PRINT("hit rate = " << hitRate);
                    }
                    else
                    {
                        //PRINT("hit rate = " << 0);
                        //continue;
                    }
                    PRINT(" ");
                    
                    if (hitRate >= threshold)
                    {
                        // the conditional statement counts the number of open page switching of a command queue if the last page policy was close page
                        if (commandQueue.bankRowBufferPolicy[i][j] == ClosePage)
                        {
                            distributedNumberOfOpenPageSwitching[i][j]++;
                        }
                        
                        commandQueue.bankRowBufferPolicy[i][j] = OpenPage;
                        PRINT("Row Buffer Policy of bank[" << i << "][" << j << "] is Open Page"   );
                    }
                    else
                    {
                        // the conditional statement counts the number of close page switching of a command queue if the last page policy was open page
                        if (commandQueue.bankRowBufferPolicy[i][j] == OpenPage)
                        {
                            distributedNumberOfClosePageSwitching[i][j]++;
                        }
                        commandQueue.bankRowBufferPolicy[i][j] = ClosePage;
                        commandQueue.switchedToClosePage[i][j] = true;
                        PRINT("Row Buffer Policy of bank[" << i << "][" << j << "] is Close Page"   );
                    }
                     
                }
            }
        }
        else
        {
            // As we have unified page policy for all command queues, therefore, we count
            // how many command queues favor open page and how many command queues favor close page.
            // After that, we compare the two numbers and decide which page policy should be chosen.
            int openPageHits = 0;
            int closePageHits = 0;
            //PRINT("Threshold = " << threshold);
            
            for (size_t i=0; i<NUM_RANKS; i++)
            {
                for (size_t j=0; j<NUM_BANKS; j++)
                {
                    double hitRate;
                    if (commandQueue.bankAccessCounters[i][j] == 0)
                    {
                        //closePageHits++;
                        hitRate = 0;
                        //continue;
                    }
                    else
                    {
                        hitRate = (double)commandQueue.bankHitCounters[i][j] / (double)commandQueue.bankAccessCounters[i][j];

                    }
                    
                    //PRINT("commandQueue.bankHitCounters = " << commandQueue.bankHitCounters[i][j]);
                    //PRINT("commandQueue.bankAccessCounters = " << commandQueue.bankAccessCounters[i][j]);
                    /*
                    PRINT("hit rate: " << hitRate);
                    */
                    if (hitRate >= threshold)
                    {
                        openPageHits++;
                    }
                    else
                    {
                        closePageHits++;
                    }
                }
            }
            
            if (closePageHits >= openPageHits)
            {
                // the conditional statement counts the number of close page switching if the last page policy was open page
                if (rowBufferPolicy == OpenPage)
                {
                    unifiedNumberOfClosePageSwitching++;
                }
                rowBufferPolicy = ClosePage;
                PRINT("Row Buffer Policy is Close Page");
            }
            else
            {
                // the conditional statement counts the number of open page switching if the last page policy was close page
                if (rowBufferPolicy == ClosePage)
                {
                    unifiedNumberOfOpenPageSwitching++;
                }
                rowBufferPolicy = OpenPage;
                PRINT("Row Buffer Policy is Open Page");
            }

        }
    }

    
    for (size_t i=0; i<NUM_RANKS; i++)
    {
        for (size_t j=0; j<NUM_BANKS; j++)
        {
            //XXX: this means the bank list won't be printed for partial epochs
            grandTotalBankAccesses[SEQUENTIAL(i,j)] += totalReadsPerBank[SEQUENTIAL(i,j)] + totalWritesPerBank[SEQUENTIAL(i,j)];
            totalReadsPerBank[SEQUENTIAL(i,j)] = 0;
            totalWritesPerBank[SEQUENTIAL(i,j)] = 0;
            totalEpochLatency[SEQUENTIAL(i,j)] = 0;
            
            commandQueue.bankHitCounters[i][j] = 0;
            commandQueue.bankAccessCounters[i][j] = 0;
            
        }
        //PRINT("current clock cycle " << currentClockCycle);
        // comment out resetting energy after each epoch so that we can get the overall energy at the end of simulation
        //burstEnergy[i] = 0;
        //actpreEnergy[i] = 0;
        //refreshEnergy[i] = 0;
        //backgroundEnergy[i] = 0;
        //totalReadsPerRank[i] = 0;
        //totalWritesPerRank[i] = 0;
    }
}

//prints statistics at the end of an epoch or  simulation
void MemoryController::printStats(bool finalStats)
{
    unsigned myChannel = parentMemorySystem->systemID;
    
    //if we are not at the end of the epoch, make sure to adjust for the actual number of cycles elapsed
    
    uint64_t cyclesElapsed = (currentClockCycle % EPOCH_LENGTH == 0) ? EPOCH_LENGTH : currentClockCycle % EPOCH_LENGTH;
    unsigned bytesPerTransaction = (JEDEC_DATA_BUS_BITS*BL)/8;
    uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
    double secondsThisEpoch = (double)cyclesElapsed * tCK * 1E-9;
    
    // only per rank
    vector<double> backgroundPower = vector<double>(NUM_RANKS,0.0);
    vector<double> burstPower = vector<double>(NUM_RANKS,0.0);
    vector<double> refreshPower = vector<double>(NUM_RANKS,0.0);
    vector<double> actprePower = vector<double>(NUM_RANKS,0.0);
    vector<double> averagePower = vector<double>(NUM_RANKS,0.0);
    
    // per bank variables
    vector<double> averageLatency = vector<double>(NUM_RANKS*NUM_BANKS,0.0);
    vector<double> bandwidth = vector<double>(NUM_RANKS*NUM_BANKS,0.0);
    
    double totalBandwidth=0.0;
    for (size_t i=0;i<NUM_RANKS;i++)
    {
        for (size_t j=0; j<NUM_BANKS; j++)
        {
            bandwidth[SEQUENTIAL(i,j)] = (((double)(totalReadsPerBank[SEQUENTIAL(i,j)]+totalWritesPerBank[SEQUENTIAL(i,j)]) * (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) / secondsThisEpoch;
            averageLatency[SEQUENTIAL(i,j)] = ((float)totalEpochLatency[SEQUENTIAL(i,j)] / (float)(totalReadsPerBank[SEQUENTIAL(i,j)])) * tCK;
            totalBandwidth+=bandwidth[SEQUENTIAL(i,j)];
            totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i,j)];
            totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];
        }
    }
#ifdef LOG_OUTPUT
    dramsim_log.precision(3);
    dramsim_log.setf(ios::fixed,ios::floatfield);
#else
    cout.precision(3);
    cout.setf(ios::fixed,ios::floatfield);
#endif
    
    PRINT( " =======================================================" );
    PRINT( " ============== Printing Statistics [id:"<<parentMemorySystem->systemID<<"]==============" );
    PRINTN( "   Total Return Transactions : " << totalTransactions );
    PRINT( " ("<<totalBytesTransferred <<" bytes) aggregate average bandwidth "<<totalBandwidth<<"GB/s");
    
    
    csvOut.getOutputStream() << endl;
    csvOut.getOutputStream() <<" =======================================================" << endl;
    csvOut.getOutputStream() << " ============== Printing Statistics [id:"<<parentMemorySystem->systemID<<"]==============";
    csvOut.getOutputStream() << "   Total Return Transactions : " << totalTransactions;
    csvOut.getOutputStream() << " ("<<totalBytesTransferred <<" bytes) aggregate average bandwidth "<<totalBandwidth<<"GB/s";
    
    
    double totalAggregateBandwidth = 0.0;	
    for (size_t r=0;r<NUM_RANKS;r++)
    {
        
        PRINT( "      -Rank   "<<r<<" : ");
        
        PRINTN( "        -Reads  : " << totalReadsPerRank[r]);
        PRINT( " ("<<totalReadsPerRank[r] * bytesPerTransaction<<" bytes)");
        PRINTN( "        -Writes : " << totalWritesPerRank[r]);
        PRINT( " ("<<totalWritesPerRank[r] * bytesPerTransaction<<" bytes)");
        
        csvOut.getOutputStream() << endl;
        csvOut.getOutputStream() <<"      -Rank   "<<r<<" : " << endl;
        
        csvOut.getOutputStream() <<"        -Reads  : " << totalReadsPerRank[r];
        
        csvOut.getOutputStream() <<" ("<<totalReadsPerRank[r] * bytesPerTransaction<<" bytes)" << endl;
        
        csvOut.getOutputStream() <<"        -Writes : " << totalWritesPerRank[r];
        csvOut.getOutputStream() <<" ("<<totalWritesPerRank[r] * bytesPerTransaction<<" bytes)" << endl;
        
        for (size_t j=0;j<NUM_BANKS;j++)
        {
            PRINT( "        -Bandwidth / Latency  (Bank " <<j<<"): " <<bandwidth[SEQUENTIAL(r,j)] << " GB/s\t\t" <<averageLatency[SEQUENTIAL(r,j)] << " ns");
            
            csvOut.getOutputStream() << endl;
            csvOut.getOutputStream() << "        -Bandwidth / Latency  (Bank " <<j<<"): " <<bandwidth[SEQUENTIAL(r,j)] << " GB/s\t\t" <<averageLatency[SEQUENTIAL(r,j)] << " ns";
            csvOut.getOutputStream() << endl;
            
        }
        
        // factor of 1000 at the end is to account for the fact that totalEnergy is accumulated in mJ since IDD values are given in mA
        
        backgroundPower[r] = ((double)backgroundEnergy[r] / (double)(cyclesElapsed)) * Vdd / 1000.0;
        burstPower[r] = ((double)burstEnergy[r] / (double)(cyclesElapsed)) * Vdd / 1000.0;
        refreshPower[r] = ((double) refreshEnergy[r] / (double)(cyclesElapsed)) * Vdd / 1000.0;
        actprePower[r] = ((double)actpreEnergy[r] / (double)(cyclesElapsed)) * Vdd / 1000.0;
        averagePower[r] = ((backgroundEnergy[r] + burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r]) / (double)cyclesElapsed) * Vdd / 1000.0;
        
        cout << endl;
        cout << "Total Energy       : " << backgroundEnergy[r] + burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r] << endl;
        cout << "Background Energy      : "<< backgroundEnergy[r] << endl;
        cout << "Act/Pre Energy     : " << actpreEnergy[r] << endl;
        cout << "Burst Energy       : " << burstEnergy[r] << endl;
        cout <<"Refresh Energy      : " << refreshEnergy[r] << endl;
        cout << endl;
        
        cout << "   Average Power (watts)     : " << averagePower[r] << endl;
        cout << "     -Background (watts)     : " << backgroundPower[r] << endl;
        cout << "     -Act/Pre    (watts)     : " << actprePower[r] << endl;
        cout << "     -Burst      (watts)     : " << burstPower[r] << endl;
        cout << "     -Refresh    (watts)     : " << refreshPower[r] << endl;
        
        if ((*parentMemorySystem->ReportPower)!=NULL)
        {
            (*parentMemorySystem->ReportPower)(backgroundPower[r],burstPower[r],refreshPower[r],actprePower[r]);
        }
        
        PRINT( " == Power Data for Rank        " << r );
        PRINT( "   Average Power (watts)     : " << averagePower[r] );
        PRINT( "     -Background (watts)     : " << backgroundPower[r] );
        PRINT( "     -Act/Pre    (watts)     : " << actprePower[r] );
        PRINT( "     -Burst      (watts)     : " << burstPower[r]);
        PRINT( "     -Refresh    (watts)     : " << refreshPower[r] );
        if (VIS_FILE_OUTPUT)
        {
            csvOut.getOutputStream() << endl;
            csvOut.getOutputStream() << "Total Energy       : " << backgroundEnergy[r] + burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r] << endl;
            csvOut.getOutputStream() << "Background Energy      : "<< backgroundEnergy[r] << endl;
            csvOut.getOutputStream() << "Act/Pre Energy     : " << actpreEnergy[r] << endl;
            csvOut.getOutputStream() << "Burst Energy       : " << burstEnergy[r] << endl;
            csvOut.getOutputStream() <<"Refresh Energy      : " << refreshEnergy[r] << endl;
            
            csvOut.getOutputStream() << endl;
            csvOut.getOutputStream() << " == Power Data for Rank=" << r <<endl;
            csvOut.getOutputStream() << " Average Power (watts)=" <<averagePower[r] <<endl;
            csvOut.getOutputStream() << " -Background (watts)=" <<backgroundPower[r] <<endl;
            csvOut.getOutputStream() << " -Act/Pre    (watts)=" <<actprePower[r] <<endl;
            csvOut.getOutputStream() << " -Burst      (watts)=" <<burstPower[r] <<endl;
            csvOut.getOutputStream() << " -Refresh    (watts)=" <<refreshPower[r] <<endl;
            csvOut.getOutputStream() << endl;
            csvOut.getOutputStream() << " Total Number of cylces=" <<currentClockCycle <<endl;
            csvOut.getOutputStream() << endl;
            
            if (HYBRID_PAGE_POLICY_FLAG == true)
            {
                if (DISTRIBUTED_PAGE_POLICY_FLAG == true)
                {
                    /*
                    // they are vectors
                    csvOut.getOutputStream() << " Number of Open Page switching= " << distributedNumberOfOpenPageSwitching <<endl;
                    csvOut.getOutputStream() << " Number of Close Page switching=" << distributedNumberOfClosePageSwitching <<endl;
                    csvOut.getOutputStream() << endl;
                     */
                }
                else
                {
                    csvOut.getOutputStream() << " Number of Open Page switching= " << unifiedNumberOfOpenPageSwitching <<endl;
                    csvOut.getOutputStream() << " Number of Close Page switching=" << unifiedNumberOfClosePageSwitching <<endl;
                    csvOut.getOutputStream() << endl;
                }
            }

            
            
        }
        
        
        if (VIS_FILE_OUTPUT)
        {
            //	cout << "c="<<myChannel<< " r="<<r<<"writing to csv out on cycle "<< currentClockCycle<<endl;
            // write the vis file output
            csvOut << CSVWriter::IndexedName("Background_Power",myChannel,r) <<backgroundPower[r];
            csvOut << CSVWriter::IndexedName("ACT_PRE_Power",myChannel,r) << actprePower[r];
            csvOut << CSVWriter::IndexedName("Burst_Power",myChannel,r) << burstPower[r];
            csvOut << CSVWriter::IndexedName("Refresh_Power",myChannel,r) << refreshPower[r];
            double totalRankBandwidth=0.0;
            for (size_t b=0; b<NUM_BANKS; b++)
            {
                csvOut << CSVWriter::IndexedName("Bandwidth",myChannel,r,b) << bandwidth[SEQUENTIAL(r,b)];
                totalRankBandwidth += bandwidth[SEQUENTIAL(r,b)];
                totalAggregateBandwidth += bandwidth[SEQUENTIAL(r,b)];
                csvOut << CSVWriter::IndexedName("Average_Latency",myChannel,r,b) << averageLatency[SEQUENTIAL(r,b)];
            }
            csvOut << CSVWriter::IndexedName("Rank_Aggregate_Bandwidth",myChannel,r) << totalRankBandwidth; 
            csvOut << CSVWriter::IndexedName("Rank_Average_Bandwidth",myChannel,r) << totalRankBandwidth/NUM_RANKS; 
        }
        
        
    }
    if (VIS_FILE_OUTPUT)
    {
        csvOut << CSVWriter::IndexedName("Aggregate_Bandwidth",myChannel) << totalAggregateBandwidth;
        csvOut << CSVWriter::IndexedName("Average_Bandwidth",myChannel) << totalAggregateBandwidth / (NUM_RANKS*NUM_BANKS);
    }
    
    
    // only print the latency histogram at the end of the simulation since it clogs the output too much to print every epoch
    if (finalStats)
    {
        PRINT( " ---  Latency list ("<<latencies.size()<<")");
        PRINT( "       [lat] : #");
        if (VIS_FILE_OUTPUT)
        {
            csvOut.getOutputStream() << "!!HISTOGRAM_DATA"<<endl;
        }
        
        map<unsigned,unsigned>::iterator it; //
        for (it=latencies.begin(); it!=latencies.end(); it++)
        {
            PRINT( "       ["<< it->first <<"-"<<it->first+(HISTOGRAM_BIN_SIZE-1)<<"] : "<< it->second );
            if (VIS_FILE_OUTPUT)
            {
                csvOut.getOutputStream() << it->first <<"="<< it->second << endl;
            }
        }
        if (currentClockCycle % EPOCH_LENGTH == 0)
        {
            PRINT( " --- Grand Total Bank usage list");
            for (size_t i=0;i<NUM_RANKS;i++)
            {
                PRINT("Rank "<<i<<":"); 
                for (size_t j=0;j<NUM_BANKS;j++)
                {
                    PRINT( "  b"<<j<<": "<<grandTotalBankAccesses[SEQUENTIAL(i,j)]);
                }
            }
        }
        
    }
    
    
    PRINT(endl<< " == Pending Transactions : "<<pendingReadTransactions.size()<<" ("<<currentClockCycle<<")==");
    /*
     for(size_t i=0;i<pendingReadTransactions.size();i++)
     {
     PRINT( i << "] I've been waiting for "<<currentClockCycle-pendingReadTransactions[i].timeAdded<<endl;
     }
     */
#ifdef LOG_OUTPUT
    dramsim_log.flush();
#endif
    
    resetStats();
}
MemoryController::~MemoryController()
{
    //ERROR("MEMORY CONTROLLER DESTRUCTOR");
    //abort();
    for (size_t i=0; i<pendingReadTransactions.size(); i++)
    {
        delete pendingReadTransactions[i];
    }
    for (size_t i=0; i<returnTransaction.size(); i++)
    {
        delete returnTransaction[i];
    }
    
}
//inserts a latency into the latency histogram
void MemoryController::insertHistogram(unsigned latencyValue, unsigned rank, unsigned bank)
{
    totalEpochLatency[SEQUENTIAL(rank,bank)] += latencyValue;
    //poor man's way to bin things.
    latencies[(latencyValue/HISTOGRAM_BIN_SIZE)*HISTOGRAM_BIN_SIZE]++;
}
