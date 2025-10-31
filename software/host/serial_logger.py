import glob
import os
import time
from datetime import datetime, timedelta
from typing import Dict, Set
from collections import defaultdict

class SerialLogger:
    """
    Class to handle logging of all serial messages
    with distinction between sent and received messages, timestamps and colors
    Includes filtering system for repeated errors
    """
    
    # ANSI color codes for terminal
    COLORS = {
        'RESET': '\033[0m',
        'BOLD': '\033[1m',
        'GREEN': '\033[32m',    # Sent messages
        'BLUE': '\033[34m',     # Received messages
        'YELLOW': '\033[33m',   # Header/info
        'RED': '\033[31m',      # Errors
        'CYAN': '\033[36m',     # Timestamp
        'MAGENTA': '\033[35m',  # Separators
    }
    
    # Define message types and their width for alignment
    MESSAGE_TYPES = {
        'SENT': '📤 SENT',
        'RECEIVED': '📥 RECEIVED', 
        'INFO': 'ℹ️  INFO',
        'ERROR': '❌ ERROR'
    }
    
    # Error patterns to filter (avoid log spam)
    FILTERED_ERROR_PATTERNS = {
        'serial_connection': [
            'could not open port',
            'No such file or directory',
            'Device not configured',
            'read failed'
        ],
        'timeout': [
            'timeout'
        ]
    }
    
    def __init__(self, log_file_path: str = "serial_communication.log", 
                 enable_error_filtering: bool = True,
                 max_repeated_errors: int = 3):
        """
        Initialize serial logger
        
        Args:
            log_file_path (str): Log file path (will be modified to include timestamp)
            enable_error_filtering (bool): Whether to enable repeated error filtering
            max_repeated_errors (int): Maximum number of identical errors to log before filtering
        """
        self.base_log_path = log_file_path
        self.start_time = datetime.now()
        self.enable_error_filtering = enable_error_filtering
        self.max_repeated_errors = max_repeated_errors
        
        # Create unique file path with timestamp
        timestamp = self.start_time.strftime('%Y%m%d_%H%M%S')
        log_dir = os.path.dirname(log_file_path) or "logs"
        base_name = os.path.splitext(os.path.basename(log_file_path))[0]
        extension = os.path.splitext(os.path.basename(log_file_path))[1] or ".log"
        
        # Format: logs/serial_communication_20240115_143022.log
        self.log_file_path = os.path.join(log_dir, f"{base_name}_{timestamp}{extension}")
        
        # Counters for repeated errors
        self.error_counts: Dict[str, int] = defaultdict(int)
        self.last_error_time: Dict[str, datetime] = {}
        self.suppressed_errors: Set[str] = set()
        
        # Calculate maximum width for pipe alignment
        # Consider that emojis can have different widths in different terminals
        # We use fixed width based on longest text
        self.max_type_width = max(len(msg_type) for msg_type in self.MESSAGE_TYPES.values()) + 2
        
        # Create directory if it doesn't exist
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # Delete log files older than configured limit (default 10 days)
        self._purge_old_logs(log_dir, base_name, extension)

        self._create_new_log_file()

    def _create_new_log_file(self):
        """
        Create a new log file with unique name and header
        Now does NOT overwrite existing files but always creates a new one
        """
        # Create new file with header (always a new file)
        with open(self.log_file_path, 'w', encoding='utf-8') as f:
            f.write(self._generate_header())
        
        print(f"📄 New session log created: {self.log_file_path}")
    
    def _generate_header(self) -> str:
        """
        Generate log file header with recording start date/time
        """
        filter_status = "🔇 ENABLED" if self.enable_error_filtering else "🔊 DISABLED"
        
        header = f"""
{self.COLORS['MAGENTA']}{'='*80}{self.COLORS['RESET']}
{self.COLORS['BOLD']}{self.COLORS['YELLOW']}🤖 RASPBERRY PI 5 - PICO COMMUNICATION LOG{self.COLORS['RESET']}
{self.COLORS['MAGENTA']}{'='*80}{self.COLORS['RESET']}

{self.COLORS['CYAN']}📅 Recording started: {self.start_time.strftime('%d/%m/%Y %H:%M:%S')}{self.COLORS['RESET']}
{self.COLORS['CYAN']}🕒 Timestamp format: [HH:MM:SS.mmm]{self.COLORS['RESET']}
{self.COLORS['CYAN']}🛡️  Error filtering: {filter_status} (max {self.max_repeated_errors} per type){self.COLORS['RESET']}

{self.COLORS['BOLD']}Legend:{self.COLORS['RESET']}
{self.COLORS['GREEN']}{self.MESSAGE_TYPES['SENT']:<{self.max_type_width}}{self.COLORS['RESET']} - Messages sent from Raspberry Pi to PICO
{self.COLORS['BLUE']}{self.MESSAGE_TYPES['RECEIVED']:<{self.max_type_width}}{self.COLORS['RESET']} - Messages received from PICO to Raspberry Pi

{self.COLORS['MAGENTA']}{'='*80}{self.COLORS['RESET']}

"""
        return header

    def _purge_old_logs(self, directory: str, base_name: str, extension: str, keep_days: int = 10) -> None:
        """Delete logs older than *keep_days* days in the specified folder."""
        try:
            cutoff = datetime.now() - timedelta(days=keep_days)
            pattern = os.path.join(directory, f"{base_name}_*{extension}")

            for log_path in glob.glob(pattern):
                try:
                    file_time = datetime.fromtimestamp(os.path.getmtime(log_path))
                    if file_time < cutoff:
                        os.remove(log_path)
                        print(f"🧹 Removed obsolete log (> {keep_days} days): {log_path}")
                except OSError as exc:
                    print(f"⚠️ Cannot delete log {log_path}: {exc}")
        except Exception as exc:
            print(f"⚠️ Log cleanup failed: {exc}")
    
    def _get_timestamp(self) -> str:
        """
        Generate formatted timestamp for log
        """
        now = datetime.now()
        return f"{self.COLORS['CYAN']}[{now.strftime('%H:%M:%S.%f')[:-3]}]{self.COLORS['RESET']}"
    
    def _format_message_type(self, msg_type: str, color: str, bold: bool = True) -> str:
        """
        Format message type with padding to align pipes
        
        Args:
            msg_type (str): Message type (MESSAGE_TYPES key)
            color (str): ANSI color code
            bold (bool): Whether to apply bold
            
        Returns:
            str: Formatted string with padding
        """
        message_text = self.MESSAGE_TYPES[msg_type]
        bold_code = self.COLORS['BOLD'] if bold else ''
        
        # Create colored string
        colored_text = f"{color}{bold_code}{message_text}{self.COLORS['RESET']}"
        
        # Calculate necessary padding (considering emojis take more space)
        # We use an empirical formula to handle emoji width
        emoji_count = sum(1 for char in message_text if ord(char) > 127)
        effective_length = len(message_text) - emoji_count  # Emojis count as 1 but are visually wider
        
        # Add padding to align to max_type_width
        padding_needed = self.max_type_width - effective_length
        padding = ' ' * max(0, padding_needed)
        
        return f"{colored_text}{padding}"
    
    def _format_message(self, message: str, max_length: int = 100) -> str:
        """
        Format message for display, truncating if too long
        """
        if len(message) > max_length:
            return f"{message[:max_length-3]}..."
        return message
    
    def _should_filter_error(self, error_message: str) -> bool:
        """
        Determine if an error should be filtered to avoid spam
        
        Args:
            error_message (str): Error message to check
            
        Returns:
            bool: True if error should be filtered (not logged)
        """
        if not self.enable_error_filtering:
            return False
        
        # Normalize message for comparison
        error_lower = error_message.lower()
        
        # Check if it matches a filtered pattern
        for pattern_category, patterns in self.FILTERED_ERROR_PATTERNS.items():
            for pattern in patterns:
                if pattern in error_lower:
                    # Create unique key for this error type
                    error_key = f"{pattern_category}:{pattern}"
                    
                    # Increment counter
                    self.error_counts[error_key] += 1
                    current_time = datetime.now()
                    
                    # If first error of this type, always log it
                    if self.error_counts[error_key] == 1:
                        self.last_error_time[error_key] = current_time
                        return False
                    
                    # If threshold exceeded, start filtering
                    if self.error_counts[error_key] > self.max_repeated_errors:
                        # Log suppression message only first time
                        if error_key not in self.suppressed_errors:
                            self.suppressed_errors.add(error_key)
                            self._log_suppression_message(pattern, self.error_counts[error_key])
                        return True
                    
                    # Update last error timestamp
                    self.last_error_time[error_key] = current_time
                    return False
        
        # Doesn't match any filtered pattern
        return False
    
    def _log_suppression_message(self, pattern: str, count: int):
        """
        Log a message indicating that errors of a certain type are being suppressed
        """
        timestamp = self._get_timestamp()
        type_text = self._format_message_type('INFO', self.COLORS['YELLOW'], False)
        suppression_msg = f"🔇 Errors containing '{pattern}' suppressed after {count} occurrences"
        
        log_entry = f"{timestamp} {type_text}| {suppression_msg}\n"
        
        with open(self.log_file_path, 'a', encoding='utf-8') as f:
            f.write(log_entry)
        
        print(log_entry.strip())
    
    def log_sent_message(self, message: str):
        """
        Log a message sent to PICO
        
        Args:
            message (str): Message sent
        """
        timestamp = self._get_timestamp()
        formatted_msg = self._format_message(message)
        type_text = self._format_message_type('SENT', self.COLORS['GREEN'], True)
        
        log_entry = f"{timestamp} {type_text}| {formatted_msg}\n"
        
        # Write to file
        with open(self.log_file_path, 'a', encoding='utf-8') as f:
            f.write(log_entry)
        
        # Optional: also print to console for debug
        print(log_entry.strip())
    
    def log_received_message(self, message: str):
        """
        Log a message received from PICO
        
        Args:
            message (str): Message received
        """
        # Ignore empty messages
        if not message.strip():
            return
            
        timestamp = self._get_timestamp()
        formatted_msg = self._format_message(message)
        type_text = self._format_message_type('RECEIVED', self.COLORS['BLUE'], True)
        
        log_entry = f"{timestamp} {type_text}| {formatted_msg}\n"
        
        # Write to file
        with open(self.log_file_path, 'a', encoding='utf-8') as f:
            f.write(log_entry)
        
        # Optional: also print to console for debug
        print(log_entry.strip())
    
    def log_info(self, message: str):
        """
        Log an informational message
        
        Args:
            message (str): Informational message
        """
        timestamp = self._get_timestamp()
        type_text = self._format_message_type('INFO', self.COLORS['YELLOW'], False)
        
        log_entry = f"{timestamp} {type_text}| {message}\n"
        
        with open(self.log_file_path, 'a', encoding='utf-8') as f:
            f.write(log_entry)
        
        print(log_entry.strip())
    
    def log_error(self, message: str):
        """
        Log an error message with optional filtering
        
        Args:
            message (str): Error message
        """
        # Check if error should be filtered
        if self._should_filter_error(message):
            return  # Don't log filtered errors
        
        timestamp = self._get_timestamp()
        type_text = self._format_message_type('ERROR', self.COLORS['RED'], True)
        
        log_entry = f"{timestamp} {type_text}| {message}\n"
        
        with open(self.log_file_path, 'a', encoding='utf-8') as f:
            f.write(log_entry)
        
        print(log_entry.strip())
    
    def get_error_statistics(self) -> Dict[str, int]:
        """
        Return statistics on filtered errors
        
        Returns:
            Dict[str, int]: Dictionary with error counts by type
        """
        return dict(self.error_counts)
    
    def close_session(self):
        """
        Close logging session adding footer with statistics
        """
        end_time = datetime.now()
        duration = end_time - self.start_time
        
        # Calculate error statistics
        total_errors = sum(self.error_counts.values())
        filtered_errors = sum(count for count in self.error_counts.values() if count > self.max_repeated_errors)
        
        footer = f"""
{self.COLORS['MAGENTA']}{'='*80}{self.COLORS['RESET']}
{self.COLORS['YELLOW']}🏁 Recording ended: {end_time.strftime('%d/%m/%Y %H:%M:%S')}{self.COLORS['RESET']}
{self.COLORS['YELLOW']}⏱️  Session duration: {duration}{self.COLORS['RESET']}
{self.COLORS['YELLOW']}🛡️  Total errors detected: {total_errors}{self.COLORS['RESET']}
{self.COLORS['YELLOW']}🔇 Errors filtered: {filtered_errors}{self.COLORS['RESET']}
{self.COLORS['CYAN']}📄 Session file: {os.path.basename(self.log_file_path)}{self.COLORS['RESET']}
{self.COLORS['MAGENTA']}{'='*80}{self.COLORS['RESET']}
"""
        
        with open(self.log_file_path, 'a', encoding='utf-8') as f:
            f.write(footer)
        
        print(f"📄 Log session completed: {self.log_file_path}") 
