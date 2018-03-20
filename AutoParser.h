/** \file
 * Tokens used in our scripting language
 */

#ifndef AUTOPARSER_H
#define AUTOPARSER_H

// any line in the parser file that begins with a space or a # is skipped

const char sComment = '#';
const char szDelimiters[] = " ,[]()\r\n\t";

///N - doesn't need a response; R - needs a response; _ - contained within auto thread
typedef enum AUTO_COMMAND_TOKENS
{
	AUTO_TOKEN_MODE,				//!<	mode block number, number(integer)
	AUTO_TOKEN_DEBUG,				//!<	debug mode, 0 = off, 1 = on
	AUTO_TOKEN_MESSAGE,				//!<	print debug message
	AUTO_TOKEN_BEGIN,				//!<	mark beginning of program
	AUTO_TOKEN_END,					//!<	mark end of program
	AUTO_TOKEN_DELAY,				//!<	delay (seconds:float)
	AUTO_TOKEN_MOVE,				//!<N	move (left & right PWM - float)
	AUTO_TOKEN_MMOVE,				//!<R	mmove <speed> <inches:float>  <timeout:float>
	AUTO_TOKEN_VMOVE,
	AUTO_TOKEN_TURN,				//!<R	turn <degrees:float> <timeout:float>
	AUTO_TOKEN_ELEVATOR,			//!<R	elevator <INTAKE|SWITCH|SCALEHI|SCALELO|STOW> <timeout:float>
	AUTO_TOKEN_CLAW,                //!<R	claw <IN|OUT|STOP>
	AUTO_TOKEN_ARM,                 //!<R	arm <PINCH|RELEASE>
	AUTO_TOKEN_SPUNCH,				//!<R   spunch <LEFT|RIGHT> <speed> <inches:float> <timeout:float>
	AUTO_TOKEN_LAST
} AUTO_COMMAND_TOKENS;

#endif  // AUTOPARSER_H
