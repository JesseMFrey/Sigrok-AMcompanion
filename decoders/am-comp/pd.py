##
## This file is part of the libsigrokdecode project.
##
## Copyright (C) 2014 Jens Steinhauser <jens.steinhauser@gmail.com>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd
import warnings

class ChannelError(Exception):
    pass

commands = {
#   val : 'name'
    0x01: 'SETUP',
    0x02: 'FETCH',
    0x03: 'NOTIFY',
}

flight_states = {
        0 : 'STARTUP',
        1 : 'IDLE',
        2 : 'PAD',
        3 : 'BOOST',
        4 : 'FAST',
        5 : 'COAST',
        6 : 'DROUGE',
        7 : 'MAIN',
        8 : 'LANDED',
        9 : 'INVALID',
        10: 'TEST'
}

frame_data = [
        {'type' : 'COMMAND'     ,'name' : 'COMMAND' ,'bytes' : 1},
        {'type' : 'FLIGHT_STATE','name' : 'STATE'   ,'bytes' : 1},
        {'type' : 'uval'        ,'name' : 'TICK'    ,'bytes' : 2, 'scale' : 0.01 , 'units' : 's'},
        {'type' : 'num'         ,'name' : 'SERIAL'  ,'bytes' : 2},
        {'type' : 'num'         ,'name' : 'FLIGHT'  ,'bytes' : 2},
        {'type' : 'val'         ,'name' : 'ACCEL'   ,'bytes' : 2, 'scale' : 1/16 , 'units' : 'm/s^2'},
        {'type' : 'val'         ,'name' : 'SPEED'   ,'bytes' : 2, 'scale' : 1/16 , 'units' : 'm/s'},
        {'type' : 'val'         ,'name' : 'HEIGHT'  ,'bytes' : 2, 'scale' : 1    , 'units' : 'm'},
        {'type' : 'num'         ,'name' : 'MOTOR'   ,'bytes' : 2}
]

companion_setup = [
        {'type' : 'num'         ,'name' : 'ID'      ,'bytes' : 2},
        {'type' : 'num'         ,'name' : 'ID_INV'  ,'bytes' : 2},
        {'type' : 'uval'        ,'name' : 'UPDATE'  ,'bytes' : 1, 'scale' : 0.01 , 'units' : 's'},
        {'type' : 'num'         ,'name' : 'TELM'    ,'bytes' : 1}
]

def sign_extend(value, nb):
    sign_bit = 1 << (nb*8 - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)


class Decoder(srd.Decoder):
    api_version = 3
    id = 'am-comp'
    name = 'AMcomp'
    longname = 'AltusMetrum Companion'
    desc = 'Rocket altimiter companion interface'
    license = 'gplv2+'
    inputs = ['spi']
    outputs = ['Command']
    options = (
    )
    annotations = (
        # Sent from the host to the chip.
        ('cmd', 'Commands sent to the device'),
        ('tx-data', 'Payload sent to the device'),

        # Returned by the chip.
        ('register', 'Registers read from the device'),
        ('rx-data', 'Payload read from the device'),

        ('warning', 'Warnings'),
    )
    ann_cmd = 0
    ann_tx = 1
    ann_reg = 2
    ann_rx = 3
    ann_warn = 4
    annotation_rows = (
        ('commands', 'Commands', (ann_cmd, ann_tx)),
        ('responses', 'Responses', (ann_reg, ann_rx)),
        ('warnings', 'Warnings', (ann_warn,)),
    )

    framesize=16
    respsize=6

    def __init__(self):
        self.reset()

    def reset(self):
        self.next()
        self.requirements_met = True
        self.cs_was_released = False

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)

    def warn(self, pos, msg):
        '''Put a warning message 'msg' at 'pos'.'''
        self.put(pos[0], pos[1], self.out_ann, [self.ann_warn, [msg]])

    def putp(self, pos, ann, msg):
        '''Put an annotation message 'msg' at 'pos'.'''
        self.put(pos[0], pos[1], self.out_ann, [ann, [msg]])

    def next(self):
        '''Resets the decoder after a complete command was decoded.'''

        # The current command, and the minimum and maximum number
        # of data bytes to follow.
        self.cmd = None

        # Used to collect the bytes after the command byte
        # (and the start/end sample number).
        self.mb = []
        self.mb_s = -1
        self.mb_e = -1

        self.entry_itr=iter(frame_data)
        self.entry_bytes=0
        self.decode_value=0
        self.frame_entry=next(self.entry_itr)
        self.decode_state='FRAME_DECODE'

    def mosi_bytes(self):
        '''Returns the collected MOSI bytes of a multi byte command.'''
        return [b[0] for b in self.mb]

    def miso_bytes(self):
        '''Returns the collected MISO bytes of a multi byte command.'''
        return [b[1] for b in self.mb]

    def decode_command(self, b):
        '''Decodes the command byte 'b' '''
        c = self.parse_command(b)
        self.cmd = c

    def format_command(self):
        '''Returns the label for the current command.'''
        if self.cmd == 'R_REGISTER':
            reg = regs[self.dat][0] if self.dat in regs else 'unknown register'
            return 'Cmd R_REGISTER "{}"'.format(reg)
        else:
            return 'Cmd {}'.format(self.cmd)

    def parse_command(self, b):
        '''Parses the command byte.

        Returns the name of the command
        '''

        if(b not in commands):
            return None
        else:
            return commands[b]

    def decode_frame_byte(self,value,pos):


        if(self.decode_state=='COMPLETE'):
            self.warn(pos,'Unexpected Data')
            return

        #check if this is the first entry byte
        if(self.entry_bytes==0):
            #save start position
            self.entry_start_pos=pos[0]


        byte_rank=self.entry_bytes
        self.decode_value|=value<<(8*byte_rank)
        self.entry_bytes+=1

        if(self.entry_bytes == self.frame_entry['bytes']):
            if(self.frame_entry['type']=='COMMAND'):
                #command value decode from commands
                self.decode_command(self.decode_value)
                if(self.cmd):
                    text = f"{self.cmd}"
                else:
                    text = "Unknown Command"
            elif(self.frame_entry['type']=='FLIGHT_STATE'):
                #flight state decode from flight_states
                if(self.decode_value in flight_states):
                    text = f"{self.frame_entry['name']} =  {flight_states[self.decode_value]}"
                else:
                    text = f"invalid flight state"
            elif(self.frame_entry['type']=='uval'):
                #Unsigned scaled value with units
                value=self.decode_value*self.frame_entry['scale']
                text = f"{self.frame_entry['name']} = {value} {self.frame_entry['units']}"
            elif(self.frame_entry['type']=='val'):
                #Signed scaled value with units
                value=sign_extend(self.decode_value,self.frame_entry['bytes'])*self.frame_entry['scale']
                text = f"{self.frame_entry['name']} = {value} {self.frame_entry['units']}"
            elif(self.frame_entry['type']=='num'):
                #number, no scaling no sign
                text = f"{self.frame_entry['name']} = {str(self.decode_value)}"
            else:
                #unknown type should not happen
                warnings.warn(f"Unknown entry tpye {self.frame_entry['type']}")
                text = 'Internal Error'

            #reset decode values
            self.decode_value=0
            self.entry_bytes=0
            #check decode state
            if(self.decode_state=='FRAME_DECODE'):
                self.putp((self.entry_start_pos,pos[1]),self.ann_cmd,text)
                
                try:
                    self.frame_entry=next(self.entry_itr)
                except StopIteration:
                    #done decoding frame, see what's next
                    if(self.cmd=='SETUP'):
                        self.entry_itr=iter(companion_setup)
                        self.frame_entry=next(self.entry_itr)
                        self.decode_state='SETUP_DECODE'
                        #initialize companion ID values
                        self.companion_id=0
                        self.companion_id_inv=0
                    elif(self.cmd=='FETCH'):
                        self.frame_entry={'type' : 'num'         ,'name' : 'TLM'      ,'bytes' : 2}
                        self.entry_itr=None
                        self.decode_state='TLM_DECODE'
                    else:
                        #unknown command or notify, no more bytes to decode
                        self.entry_itr=None
                        self.decode_state='COMPLETE'
            elif(self.decode_state=='SETUP_DECODE'):
                self.putp((self.entry_start_pos,pos[1]),self.ann_rx,text)
                if(self.frame_entry['name'] == 'ID'):
                    self.companion_id_start=self.entry_start_pos
                    self.companion_id=self.decode_value
                elif(self.frame_entry['name'] == 'ID_INV'):
                    if(self.companion_id != (0xFFFF ^ self.decode_value)):
                        self.warn((self.companion_id_start,pos[1]),'Companion ID mismatch')
                try:
                    self.frame_entry=next(self.entry_itr)
                except StopIteration:
                    #done decoding, no more bytes to decode
                    self.entry_itr=None
                    self.decode_state='COMPLETE'
            elif(self.decode_state=='TLM_DECODE'):
                self.putp((self.entry_start_pos,pos[1]),self.ann_rx,text)
                #all TLM values are the same, dont iterate






    def decode(self, ss, es, data):
        if not self.requirements_met:
            return

        ptype, data1, data2 = data


        if ptype == 'CS-CHANGE':
            if data1 is None:
                if data2 is None:
                    self.requirements_met = False
                    raise ChannelError('CS# pin required.')
                elif data2 == 1:
                    self.cs_was_released = True

            if data1 == 0 and data2 == 1:
                # Rising edge, the complete command is transmitted, process
                # the bytes that were send after the command byte.
                self.next()
                self.cs_was_released = True
        elif ptype == 'DATA' and self.cs_was_released:
            mosi, miso = data1, data2
            pos = (ss, es)

            if miso is None or mosi is None:
                self.requirements_met = False
                raise ChannelError('Both MISO and MOSI pins required.')

            self.decode_frame_byte(mosi,pos)
