#define __USE_XOPEN
#define _GNU_SOURCE

#include <stdio.h>
#include <time.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

#define PNBLL_SEND_PERIOD 5
#define NBL_DATE_LENGTH 8
#define NBL_TIME_LENGTH 6
#define CHAR_BUF_SIZE 128//enough for polling port at 4800 bps every 100 ms
#define NMEA_MAX_LENGTH 79
#define BAUDRATE B4800   // for cross-compiling
//#define BAUDRATE B115200   // for test on PC
#define MODEMDEVICE "/dev/ttyS0" //on board
//#define MODEMDEVICE "/dev/ttyUSB0" //on PC
#define POWERSTATE_FILE "/var/power.state"
#define ONOFF_FILE "/system/time.table"
//#define ONOFF_FILE "/var/time.table"
#define DEFAULT_POWERON_TIME "020000"
#define DEFAULT_POWEROFF_TIME "050000"

int port_fd = -1;
FILE * timetable_fp;
int end_of_loop = 0;
char rx_nmea_line[NMEA_MAX_LENGTH];
char tx_nmea_line[NMEA_MAX_LENGTH];
int rx_idx=0;
enum _rx_state {BUF_EMPTY,PARTIAL,LINE_READY} rx_state = BUF_EMPTY;

unsigned char NMEAChecksum(char* s)//use %02X format
{
    unsigned char c = 0;
    // Initial $ is omitted from checksum, if present ignore it.
    if (*s == '$')
        ++s;

    while (*s != '\0' && *s != '*')
        c ^= *s++;
    return c;
}

int get_nmea_field(char** field, char* sentence, int number)
{//Return value: length of the field. "field" will point at its first character
    int field_start=-1, delim_found=0;
    int ret=-1;
    *field=NULL;
    for(int i=0;i<NMEA_MAX_LENGTH;++i)
    {
        if(sentence[i]==',' || sentence[i]=='*')
        {
            ++delim_found;
            if((sentence[i]=='*') && (delim_found<=number)) //There s too few fields in the sentence
                goto on_error;
            if((delim_found == number) && (sentence[i]==','))//it's start of required field
            {
                *field=&sentence[i+1];
                field_start=i+1;
            }
            if(delim_found == (number+1)) //it's end of required fild
            {
                if(field_start>=0)
                    ret=i-field_start;
                else
                    *field=NULL;
                return ret;
            }
        }
    }
    //Required field was not found in buffer
on_error:
    *field=NULL;
    return ret;
}

void reset_rx_sentence(void)
{
    rx_nmea_line[0]='\0';
    rx_idx=0;
    rx_state=BUF_EMPTY;
}

void sig_handler(int sig)
{
    if(sig == SIGINT)
    {
        printf("Exiting by SIGINT\n");
        end_of_loop = 1;
        close(port_fd);
    }
}



int main(void)
{
    struct termios newt;
    int chars_read;
    char read_buffer[CHAR_BUF_SIZE];
    char parse_buffer[32];
    char command[128];
    char tx_checksum[6];
    char* parser;
    char* _date;
    char* _time;
    struct tm tm;
    struct tm* ptm;
    int float_len;

    time_t ps_time, now, last_nmeasent;
    static double v1,i1,v2,i2;
    double timediff;
    char powerontime[7];
    char powerofftime[7];

    signal(SIGINT, sig_handler);

    size_t tt_len = 0;
    ssize_t tt_read;
    char* tt_line = NULL;

    port_fd = open(MODEMDEVICE, O_RDWR | O_NONBLOCK);
    if (port_fd >= 0)
    {
        tcgetattr(port_fd, &newt);
        cfsetispeed(&newt,BAUDRATE);
        cfsetospeed(&newt,BAUDRATE);
        newt.c_iflag &= ~IGNBRK;         
        newt.c_iflag &= ~(IXON | IXOFF | IXANY); 
        newt.c_oflag = 0;               
        newt.c_cflag |= (CLOCAL | CREAD);               
        newt.c_cflag |= CS8;                       
        newt.c_cflag &= ~(PARENB | PARODD);         
        newt.c_cflag &= ~CSTOPB;                   
        newt.c_lflag = 0;                            
        newt.c_cc[VMIN]  = 0; 
        newt.c_cc[VTIME] = 0; 
        tcsetattr(port_fd, TCSANOW, &newt);
        last_nmeasent=time(NULL);

        while(end_of_loop == 0)//until reception of SIGINT, read port every 100 ms
        {
            usleep(100000);
            
            now=time(NULL);
            timediff=difftime(now, last_nmeasent);
            if( (timediff>PNBLL_SEND_PERIOD) || (timediff<0) )
            {
                /* We must to build and send $PNBLL sentence. Its format:
                *           1      2      3 4      5      6
                *           |      |      | |      |      |
                *    $PNBLL,hhmmss,ddmmyy,A,hhmmss,hhmmss*hh
                *    
                *    
                *    1) Time (UTC)
                *    2) Date
                *    3) Time&Date status: A - synchronized with NTP, V - not syncronized
                *    4) Turn on time (UTC)
                *    5) Turn off time (UTC)
                *    6) Checksum
                */
                timetable_fp = fopen(ONOFF_FILE, "r");
                if(timetable_fp == NULL)
                {
                    sprintf(powerontime,DEFAULT_POWERON_TIME);
                    sprintf(powerofftime,DEFAULT_POWEROFF_TIME);
                }
                else
                {
                    if((tt_read = getline(&tt_line, &tt_len, timetable_fp)) >= sizeof(powerontime)-1)
                    {
                        snprintf(powerontime,sizeof(powerontime),"%s",tt_line);
                        if((tt_read = getline(&tt_line, &tt_len, timetable_fp)) >= sizeof(powerofftime)-1)
                            snprintf(powerofftime,sizeof(powerofftime),"%s",tt_line);
                        else
                            sprintf(powerofftime,DEFAULT_POWEROFF_TIME);
                    }
                    else
                    {
                        sprintf(powerontime,DEFAULT_POWERON_TIME);
                        sprintf(powerofftime,DEFAULT_POWEROFF_TIME);
                    }
                }
                fclose(timetable_fp);
                ptm=gmtime(&now);
                strftime(parse_buffer, 20, "%H%M%S,%d%m%Y", ptm);
                snprintf(tx_nmea_line,NMEA_MAX_LENGTH-4,"$PNBLL,%s,%s,%s,%s",parse_buffer,(ptm->tm_year>119)?"A":"V",powerontime,powerofftime);
                snprintf(tx_checksum,sizeof(tx_checksum),"*%02X\r\n",NMEAChecksum(tx_nmea_line));
                strcat(tx_nmea_line,tx_checksum);
                //fprintf(stderr,"%s\n",tx_nmea_line);
                write(port_fd,tx_nmea_line,strlen(tx_nmea_line));
                tx_nmea_line[0]='\0';
                last_nmeasent=now;
            }

            chars_read=read(port_fd, &read_buffer,CHAR_BUF_SIZE);
            if(chars_read<0) reset_rx_sentence();//reset rx_nmea_line and wait next sentence
            for(int i=0;i<chars_read;++i)
            {
                if(rx_idx >= NMEA_MAX_LENGTH) reset_rx_sentence();//rx_nmea_line overflow
                switch (rx_state)
                {//If wait for sentence start skip characters another than '$'
                    case BUF_EMPTY:
                        if(read_buffer[i]=='$')
                        {
                            rx_nmea_line[0]=read_buffer[i];//Copy first symbol
                            ++rx_idx;
                            rx_state = PARTIAL;
                        }
                        else rx_idx=0;//ignore
                        break;
                    case PARTIAL:
                        if((read_buffer[i]=='\r')||(read_buffer[i]=='\n'))//Sentence ended
                        {
                            if( (rx_idx>8) && (rx_nmea_line[rx_idx-3] == '*') )//Looks normal
                            {
                                rx_nmea_line[rx_idx]='\0';
                                rx_idx=0;
                                rx_state=LINE_READY;
                            }
                            else
                            {
                                reset_rx_sentence();
                                break;
                            }
                        }
                        else
                            rx_nmea_line[rx_idx]=read_buffer[i];//Copy symbol
                            ++rx_idx;
                        break;
                    case LINE_READY://Ignore new characters, need to parse sentence now
                    default:
                    break;
                }
            }
            if(rx_state==LINE_READY) {
                //int get_nmea_field(char* field, const char* sentence, int number)
                parser = strstr(rx_nmea_line, "$PNBLP");
                if (parser != NULL)
                {
                    /* Process PNBLP sentence. It has the following structure:
                    *           1      2      3 4     5       6     7       8
                    *           |      |      | |     |       |     |       |
                    *    $PNBLP,hhmmss,ddmmyy,A,xx.xx,xxxx.xx,xx.xx,xxxx.xx*hh
                    *    
                    *    1) Time (UTC)
                    *    2) Date
                    *    3) Time&Date status: A - synchronized with NTP, V - not syncronized
                    *    4) ACC voltage, V
                    *    5) ACC current, mA
                    *    6) Solar panel voltage, V
                    *    7) Solar panel current, mA
                    *    8) Checksum
                    */
                    if((get_nmea_field(&_time,rx_nmea_line,1)==NBL_TIME_LENGTH) && (get_nmea_field(&_date,rx_nmea_line,2)==NBL_DATE_LENGTH))
                    {
                        sprintf(parse_buffer,"%c%c:%c%c:%c%c %c%c.%c%c.%c%c%c%c",*_time,*(_time+1),*(_time+2),*(_time+3),*(_time+4),*(_time+5),
                                *_date,*(_date+1),*(_date+2),*(_date+3),*(_date+4),*(_date+5),*(_date+6),*(_date+7));
                        strptime(parse_buffer, "%H:%M:%S %d.%m.%Y", &tm);
                        ps_time = mktime(&tm) - timezone;//use UTC
                    }
                    float_len=get_nmea_field(&parser,rx_nmea_line,4);//ACC voltage
                    if(parser)
                    {
                        snprintf(parse_buffer,float_len+1,"%s",parser);
                        v1=atof(parse_buffer);
                    }
                    float_len=get_nmea_field(&parser,rx_nmea_line,5);//ACC current
                    if(parser)
                    {
                        snprintf(parse_buffer,float_len+1,"%s",parser);
                        i1=atof(parse_buffer);
                    }
                    float_len=get_nmea_field(&parser,rx_nmea_line,6);//Solar panel voltage
                    if(parser)
                    {
                        snprintf(parse_buffer,float_len+1,"%s",parser);
                        v2=atof(parse_buffer);
                    }
                    float_len=get_nmea_field(&parser,rx_nmea_line,7);//Solar panel current
                    if(parser)
                    {
                        snprintf(parse_buffer,float_len+1,"%s",parser);
                        i2=atof(parse_buffer);
                    }
                    sprintf(command, "echo \"Vacc=%.2f Iacc=%.2f Vsolar=%.2f Isolar=%.2f\" > "POWERSTATE_FILE" ", v1,i1,v2,i2);
                    system(command);
                    fprintf(stderr,"%s\n",command);
                }//~PNBLP
                reset_rx_sentence();//prepare to get next sentence
            }
        }//main loop

        close(port_fd);

        return 0;

    }//port_fd>0
    else
    {
        printf("Port cannot be opened");
        return -1;
    }
}
