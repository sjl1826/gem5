/**
 * Copyright (c) 2018 Metempsy Technology Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __CACHE_PREFETCH_ZCACHE_ARRAY_IMPL_HH__
#define __CACHE_PREFETCH_ZCACHE_ARRAY_IMPL_HH__

#include "base/intmath.hh"
#include "mem/cache/prefetch/zcache_array.hh"

namespace gem5
{

    GEM5_DEPRECATED_NAMESPACE(Prefetcher, prefetch);
    namespace prefetch
    {
        template <class Entry>
        ZCacheArray<Entry>::ZCacheArray(int assoc, int num_entries,
                                        BaseIndexingPolicy *idx_policy, replacement_policy::Base *rpl_policy,
                                        Entry const &init_value)
            : associativity(assoc), numEntries(num_entries), indexingPolicy(idx_policy),
              replacementPolicy(rpl_policy), entries(numEntries, init_value)
        {
            fatal_if(!isPowerOf2(num_entries), "The number of entries of an "
                                               "AssociativeSet<> must be a power of 2");
            fatal_if(!isPowerOf2(assoc), "The associativity of an AssociativeSet<> "
                                         "must be a power of 2");
            // Potential TODO: Verify that numEntries can be used to calculate the number of lines to use
            // Potential TODO: Figure out if associativity == ways
            // The number of associative sets is obtained by dividing numEntries by associativity.
            numLines = associativity * numEntries;

            for (unsigned int entry_idx = 0; entry_idx < (numEntries + associativity); entry_idx += 1)
            {
                Entry *entry = &entries[entry_idx];
                indexingPolicy->setEntry(entry, entry_idx);
                entry->replacementData = replacementPolicy->instantiateEntry();
            }
            swapArrSet = gm_calloc<uint32_t>(numEntries + associativity);
            swapArrWay = gm_calloc<uint32_t>(numEntries + associativity);
        }

        template <class Entry>
        Entry *
        ZCacheArray<Entry>::findEntry(Addr addr, bool is_secure) const
        {
            Addr tag = indexingPolicy->extractTag(addr);
            const std::vector<ReplaceableEntry *> selected_entries =
                indexingPolicy->getPossibleEntries(addr);

            for (const auto &location : selected_entries)
            {
                Entry *entry = static_cast<Entry *>(location);
                if ((entry->getTag() == tag) && entry->isValid() &&
                    entry->isSecure() == is_secure)
                {
                    return entry;
                }
            }
            return nullptr;
        }

        template <class Entry>
        void
        ZCacheArray<Entry>::accessEntry(Entry *entry)
        {
            replacementPolicy->touch(entry->replacementData);
        }

        template <class Entry>
        Entry *
        ZCacheArray<Entry>::findVictim(Addr addr)
        {
            // Get possible entries to be victimized
            const std::vector<ReplaceableEntry *> selected_entries =
                indexingPolicy->getPossibleEntries(addr);
            bool isValid = true;
            // Check if selected entries are all valid
            for (uint32_t i = 0; i < associativity; i++)
            {
                isValid &= selected_entries[i]->isValid();
                selected_entries[i]->setPosition(selected_entries[i]->getSet(),
                                                 selected_entries[i]->getWay(), -1);
            }

            uint32_t numCandidates = associativity;

            // BFS Algorithm to expand candidates at multiple levels
            for (const auto &location : selected_entries)
            {
                if (numCandidates >= numEntries || !isValid)
                {
                    break;
                }
                Entry *entry = static_cast<Entry *>(location);
                unsigned int entryId = (location->getSet() * associativity) + location->getWay();

                isValid &= entry->isValid();
                Addr tag = entry->getTag();
                Addr entryAddr = indexingPolicy->regenerateAddr(tag, entry);

                // Generate extra entries at next level
                const std::vector<ReplaceableEntry *> extra_entries =
                    indexingPolicy->getPossibleEntries(addr);
                for (uint32_t i = 0; i < associativity; i++)
                {
                    isValid &= extra_entries[i]->isValid();
                    entries[numCandidates]->setPosition(extra_entries[i]->getSet(),
                                                        extra_entries[i]->getWay(), entryId);
                    unsigned int entryId2 = (extra_entries[i]->getSet() * associativity) + extra_entries[i]->getWay();
                    if (entryId != entryId2)
                    {
                        numCandidates++;
                    }
                }
            }

            // Only grab the candidates from the entries
            std::vector<Entry *>::const_terator first = entries.begin();
            std::vector<Entry *>::const_terator last = entries.begin() + numCandidates;
            std::vector<Entry *> candidates(first, last);
            Entry *victim = static_cast<Entry *>(replacementPolicy->getVictim(
                candidates));

            // Index in entries can be calculated by (set*associativity) + way
            unsigned int victimIndex = (victim->getSet() * associativity) + victim->getWay();
            int index = victimIndex;
            int swapIndex = 0;
            while (index >= 0)
            {
                swapArrSet[swapIndex] = entries[index]->getSet();
                swapArrWay[swapIndex] = entries[index]->getWay();
                index = entries[index]->getParent();
                swapIndex++;
            }

            swapLen = swapIndex;

            // There is only one eviction for this replacement
            invalidate(victim);

            return victim;
        }

        template <class Entry>
        std::vector<Entry *>
        ZCacheArray<Entry>::getPossibleEntries(const Addr addr) const
        {
            std::vector<ReplaceableEntry *> selected_entries =
                indexingPolicy->getPossibleEntries(addr);
            std::vector<Entry *> entries(selected_entries.size(), nullptr);

            unsigned int idx = 0;
            for (auto &entry : selected_entries)
            {
                entries[idx++] = static_cast<Entry *>(entry);
            }
            return entries;
        }

        template <class Entry>
        void
        ZCacheArray<Entry>::insertEntry(Addr addr, bool is_secure, Entry *entry)
        {
            // TODO: Relocate all of the ancestors in the tree
            for (uint32_t i = 0; i < swapLen - 1; i++)
            {
                unsigned int index1 = (swapArrSet[i] * associativity) + swapArrWay[i];
                unsigned int index2 = (swapArrSet[i + 1] * associativity) + swapArrWay[i + 1];
                entries[index1] = entries[index2];
            }

            unsigned int lastId = (swapArrSet[swapLen - 1] * associativity) + swapArrWay[swapLen - 1];
            indexingPolicy->setEntry(entry, lastId);
            entry->insert(indexingPolicy->extractTag(addr), is_secure);
            replacementPolicy->reset(entry->replacementData);
        }

        template <class Entry>
        void
        ZCacheArray<Entry>::invalidate(Entry *entry)
        {
            entry->invalidate();
            replacementPolicy->invalidate(entry->replacementData);
        }
    } // namespace prefetch
    } // namespace gem5

#endif //__CACHE_PREFETCH_ZCACHE_ARRAY_IMPL_HH__
