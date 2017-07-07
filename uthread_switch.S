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
  movl next_thread, %eax
  movl (%eax), %esp
  movl %eax, current_thread
  movl $0, next_thread

  # Restore next_thread
  popal

 	ret				/* pop return address from stack */
