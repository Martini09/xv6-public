	.text

/* Switch from current_thread to next_thread. Make next_thread
 * the current_thread, and set next_thread to 0.
 * Use eax as a temporary register; it is caller saved.
 */
	.globl thread_switch
thread_switch:
	/* YOUR CODE HERE */
  # Save current_thread
  pushal
  movl current_thread, %eax
  movl %esp, (%eax)

  # Set current_thread = next_thread, next_thread = 0
  leal current_thread, %eax
  movl next_thread, %ecx
  movl %ecx, (%eax)
  
  leal next_thread, %eax
  movl 0x0, (%eax)

  # Restore next_thread
  popal
  movl current_thread, %esp

 	ret				/* pop return address from stack */
