const $=id=>document.getElementById(id);let busy=false;
const labels={title:'J-Link \u538b\u529b\u5b9e\u65f6\u8c03\u53c2',connect:'\u8fde\u63a5 J-Link',disconnect:'\u505c\u6b62\u8c03\u8bd5\u5e76\u6062\u590d\u9ed8\u8ba4',fastLabel:'\u63a8\u8fdb\u901f\u5ea6',retractLabel:'\u56de\u9000\u901f\u5ea6',ratioLabel:'\u51cf\u901f\u6bd4\u4f8b',write:'\u53d1\u9001\u53c2\u6570',note:'\u53d1\u9001\u540e\u7acb\u5373\u751f\u6548\uff0c\u4e0d\u4fdd\u5b58\u3002\u6b63\u5e38\u65ad\u5f00\u6216\u5173\u95ed\u65f6\u6062\u590d\u9ed8\u8ba4\u3002'};
for(const [id,text] of Object.entries(labels))$(id).textContent=text;
function state(connected){$('connect').disabled=connected;$('write').disabled=!connected;$('disconnect').disabled=!connected;$('status').textContent=connected?'J-Link \u5df2\u8fde\u63a5':'J-Link \u672a\u8fde\u63a5';}
const action=fn=>async()=>{if(busy)return;busy=true;try{await fn();}catch(e){$('log').textContent=e.message;}finally{busy=false;}};
$('connect').onclick=action(async()=>{await tuner.connect();state(true);});
$('disconnect').onclick=action(async()=>{await tuner.disconnect();state(false);$('log').textContent='\u9ed8\u8ba4\u53c2\u6570\u5df2\u6062\u590d';});
$('write').onclick=action(async()=>{const names=['fast','retract','kp','ratio'];if(names.some(n=>$(n).value.trim()===''))throw Error('\u8bf7\u586b\u5199\u56db\u4e2a\u53c2\u6570');await tuner.write(+$('profile').value,names.map(n=>+$(n).value));$('log').textContent='\u53c2\u6570\u53d1\u9001\u5e76\u6821\u9a8c\u6210\u529f';});
tuner.onLog(s=>$('log').textContent=s);tuner.onConnection(state);state(false);
