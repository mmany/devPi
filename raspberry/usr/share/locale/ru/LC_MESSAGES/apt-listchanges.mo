��    E      D  a   l      �  1   �  '   #  &   K     r  2   �  W   �       7   1  <   i     �  �   �  {   M     �  $   �  %   �  1   	  3   O	  $   �	     �	  *   �	  -   �	     
     (
  )   ?
  *   i
  #   �
     �
     �
  9   �
  '     	   ?  .   I  2   x  (   �  �   �     �  D   �     
           <  J   H     �  �   �     <      O     p  I   �  |   �  D   S  9   �  ]   �  _   0  G   �  %   �     �  <        P  7   k  q   �  2         H     i     �     �  #   �  "   �     �       �  (  Y   (  K   �  R   �  1   !  ?   S  {   �  0     `   @  ]   �  4   �  �   4  �     '   �  <   �  L   1  �   ~  i     i   w     �  U   �  [   P  E   �  (   �  G     O   c  :   �  )   �       v   +  =   �     �  [   �  [   M  F   �  �  �  <   �!  t   �!  !   5"  4   W"     �"  �   �"     .#  %  K#  0   q$  [   �$  A   �$  o   @%  �   �%  �   X&  t   �&  �   ^'  �   (  w   �(  C   4)  1   x)  ]   �)  K   *  �   T*  �   �*  I   �+  :   �+     ,,  2   E,     x,  A   �,  5   �,  %   -  $   6-        E              -   #              	   
          $          5   9       ?           @                 =      3          .   "   :         2   +   *                                >   '                       ,   ;       %       D       C       8                 7          <      &   1   )      0   6            (           /   !   4      B      A        $DISPLAY is not set, falling back to %(frontend)s %(deb)s does not exist or is not a file %(deb)s does not have '.deb' extension %(deb)s is not readable %(pkg)s: Version %(version)s has already been seen %(pkg)s: Version %(version)s is lower than version of related packages (%(maxversion)s) %s: will be newly installed --since=<version> and --show-all are mutually exclusive --since=<version> expects a path to exactly one .deb archive APT pipeline messages: APT_HOOK_INFO_FD environment variable is incorrectly defined
(Dpkg::Tools::Options::/usr/bin/apt-listchanges::InfoFD should be greater than 2). APT_HOOK_INFO_FD environment variable is not defined
(is Dpkg::Tools::Options::/usr/bin/apt-listchanges::InfoFD set to 20?) Aborting Available apt-listchanges frontends: Calling %(cmd)s to retrieve changelog Cannot find suitable user to drop root privileges Cannot read from file descriptor %(fd)d: %(errmsg)s Cannot reopen /dev/tty for stdin: %s Changes for %s Choose a frontend by entering its number:  Command %(cmd)s exited with status %(status)d Confirmation failed: %s Continue Installation? Database %(db)s does not end with %(ext)s Database %(db)s failed to load: %(errmsg)s Didn't find any valid .deb archives Do you want to continue? [Y/n]  Done Error getting user from variable '%(envvar)s': %(errmsg)s Error processing '%(what)s': %(errmsg)s Error: %s Failed to send mail to %(address)s: %(errmsg)s Found user: %(user)s, temporary directory: %(dir)s Ignoring `%s' (seems to be a directory!) Incorrect value (0) of APT_HOOK_INFO_FD environment variable.
If the warning persists after restart of the package manager (e.g. aptitude),
please check if the /etc/apt/apt.conf.d/20listchanges file was properly updated. Informational notes Invalid (non-numeric) value of APT_HOOK_INFO_FD environment variable List the changes Mailing %(address)s: %(subject)s News for %s None of the following directories is accessible by user %(user)s: %(dirs)s Packages list: Path to the seen database is unknown.
Please either specify it with --save-seen option
or pass --profile=apt to have it read from the configuration file. Reading changelogs Reading changelogs. Please wait. Received signal %d, exiting The following changes are found in the packages you are about to install: The gtk frontend needs a working python3-gi,
but it cannot be loaded. Falling back to %(frontend)s.
The error is: %(errmsg)s The mail frontend needs an e-mail address to be configured, using %s The mail frontend needs an installed 'sendmail', using %s Unable to retrieve changelog for package %(pkg)s; 'apt-get changelog' failed with: %(errmsg)s Unable to retrieve changelog for package %(pkg)s; could not run 'apt-get changelog': %(errmsg)s Unknown argument %(arg)s for option %(opt)s.  Allowed are: %(allowed)s. Unknown configuration file option: %s Unknown frontend: %s Usage: apt-listchanges [options] {--apt | filename.deb ...}
 Using default frontend: %s Will read apt pipeline messages from file descriptor %d Wrong or missing VERSION from apt pipeline
(is Dpkg::Tools::Options::/usr/bin/apt-listchanges::Version set to 2?) You can abort the installation if you select 'no'. apt-listchanges warning: %(msg)s apt-listchanges: %(msg)s apt-listchanges: Changelogs apt-listchanges: News apt-listchanges: Reading changelogs apt-listchanges: changelogs for %s apt-listchanges: news for %s press q to quit Project-Id-Version: apt-listchanges NEW
Report-Msgid-Bugs-To: apt-listchanges@packages.debian.org
PO-Revision-Date: 2018-03-04 23:48+0500
Last-Translator: Galina Anikina <merilaga@yandex.ru>
Language-Team: Debian L10n Russian <debian-l10n-russian@lists.debian.org>
Language: ru
MIME-Version: 1.0
Content-Type: text/plain; charset=UTF-8
Content-Transfer-Encoding: 8bit
X-Generator: Poedit 2.0.6
Plural-Forms: nplurals=3; plural=(n%10==1 && n%100!=11 ? 0 : n%10>=2 && n%10<=4 && (n%100<10 || n%100>=20) ? 1 : 2);
 Переменная $DISPLAY не установлена, возврат в %(frontend)s %(deb)s не существует или не является файлом %(deb)s не имеет расширения '.deb' в названии файла %(deb)s не доступен для чтения %(pkg)s: Версия %(version)s уже встречалась %(pkg)s: Версия %(version)s меньше номера версии связанных пакетов (%(maxversion)s) %s: будет установлен заново опции --since=<version> и --show-all являются взаимоисключающими --since=<версия> ожидает путь только к одному архиву .deb Сообщения, поступившие от APT: Переменная окружения APT_HOOK_INFO_FD определена некорректно
(значение Dpkg::Tools::Options::/usr/bin/apt-listchanges::InfoFD
должно быть больше чем 2). Переменная окружения APT_HOOK_INFO_FD не определена
(значение Dpkg::Tools::Options::/usr/bin/apt-listchanges::InfoFD 
установлено в 20?) Аварийное завершение Доступные просмотрщики apt-listchanges: Вызов %(cmd)s для загрузки журнала изменений Не удаётся найти подходящего пользователя для перехода от суперпользователя Не удалось прочитать из файлового дескриптора  %(fd)d: %(errmsg)s Не могу переоткрыть /dev/tty для стандартного потока ввода: %s Изменения в %s Для изменения просмотрщика введите его номер:  Команда %(cmd)s завершила работу с результатом %(status)d Произошла ошибка при подтверждении: %s Продолжить установку? База данных %(db)s не оканчивается на %(ext)s Не удалось загрузить базу данных %(db)s: %(errmsg)s Допустимые .deb архивы не найдены Хотите продолжить? [Y/n]  Выполнено Ошибка получения имени пользователя из переменной '%(envvar)s': %(errmsg)s Ошибка при обработке '%(what)s': %(errmsg)s Ошибка: %s Не удалось отправить почту по адресу %(address)s: %(errmsg)s Найден пользователь: %(user)s, временный каталог: %(dir)s Игнорируется `%s' (кажется, это каталог!) Некорректное значение (0) переменной окружения APT_HOOK_INFO_FD.
Если предупреждения появляются снова после перезапуска менеджера
пакетов (например aptitude), пожалуйста проверьте был ли правильно
обновлён файл /etc/apt/apt.conf.d/20listchanges. Замечания справочного характера Ошибочное (не числовое) значение переменной окружения APT_HOOK_INFO_FD Вывести изменения Отправка почты %(address)s: %(subject)s Новости о %s Ни один из следующих каталогов не открыт для доступа пользователю %(user)s: %(dirs)s Список пакетов: Путь до просматриваемой базы данных неизвестен.
Укажите его с помощью опции --save-seen, либо добавьте опцию 
--profile=apt, что позволит прочитать путь из файла настройки. Чтение журналов изменений Чтение журналов изменений. Пожалуйста, подождите. Получен сигнал %d, завершение работы В устанавливаемых пакетах были найдены следующие изменения: Просмотрщику GTK требуется рабочий python3-gi,
но он не был найден. Возврат в %(frontend)s.
Ошибка: %(errmsg)s Для просмотрщика почты требуется указать адрес электронной почты, используя %s Для просмотрщика почты требуется установить 'sendmail', используя %s Не удаётся получить журнал изменений пакета %(pkg)s; команда 'apt-get changelog' 
вернула следующие ошибки: %(errmsg)s Не удаётся получить журнал изменений пакета  %(pkg)s; не удалось запустить 
'apt-get changelog': %(errmsg)s Неизвестный аргумент %(arg)s опции %(opt)s. Допустимые значения: %(allowed)s. Неизвестная опция файла настройки: %s Неизвестный просмотрщик: %s Использование: apt-listchanges [опции] {--apt | имя_файла.deb ...}
 Используется просмотрщик по умолчанию: %s Будет выполняться чтение сообщений, получаемых от apt, из файлового дескриптора %d Неправильное (или отсутствует) значение VERSION в выводе apt
(значение Dpkg::Tools::Options::/usr/bin/apt-listchanges::Version равно 2?) Вы можете прервать установку, выбрав 'no'. Предупреждение от apt-listchanges: %(msg)s apt-listchanges: %(msg)s apt-listchanges: Журналы изменений apt-listchanges: Новости apt-listchanges: Чтение журналов изменений apt-listchanges: журналы изменений %s apt-listchanges: новости о %s нажмите q для выхода 